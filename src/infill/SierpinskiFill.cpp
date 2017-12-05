/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SierpinskiFill.h"

#include <algorithm> // swap 
#include <functional> // function

#include "../utils/linearAlg2D.h" // rotateAround

#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

SierpinskiFill::SierpinskiFill(const DensityProvider& density_provider, const AABB aabb, int max_depth, const coord_t line_width, bool dithering)
: dithering(dithering)
, density_provider(density_provider)
, aabb(aabb)
, line_width(line_width)
, max_depth(max_depth)
, pre_division_tree_size(calculatePreDivisionTreeSize(max_depth))
, pre_division_tree(pre_division_tree_size, SierpinskiTriangle())
{
    Point m = aabb.min;
    Point lt = Point(m.X, aabb.max.Y);
    Point rb = Point(aabb.max.X, m.Y);
    edges.emplace_back(straight, m, rb, 0);
    edges.emplace_back(diagonal, m, aabb.max, 0);
    edges.emplace_back(straight, m, lt, 0);
    
    using Node = FingerTree<SierpinskiTriangle, 2>::Node;
    pre_division_tree.getRoot()[0] = SierpinskiTriangle(rb, m, aabb.max, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, false, 1);
    pre_division_tree.getRoot()[1] = SierpinskiTriangle(lt, aabb.max, m, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, false, 1);
    for (unsigned int depth = 1; depth < max_depth; depth++)
    {
        for (Node level_it = pre_division_tree.begin(depth); level_it != pre_division_tree.end(depth); ++level_it)
        {
            SierpinskiTriangle t = *level_it;
            Point middle = (t.a + t.b) / 2;
            SierpinskiTriangle::SierpinskiDirection first_dir, second_dir;
            switch(t.dir)
            {
                case SierpinskiTriangle::SierpinskiDirection::AB_TO_BC:
                    first_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_BC;
                    second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_AB;
                    break;
                case SierpinskiTriangle::SierpinskiDirection::AC_TO_AB:
                    first_dir = SierpinskiTriangle::SierpinskiDirection::AB_TO_BC;
                    second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_BC;
                    break;
                case SierpinskiTriangle::SierpinskiDirection::AC_TO_BC:
                    first_dir = SierpinskiTriangle::SierpinskiDirection::AB_TO_BC;
                    second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_AB;
                    break;
            }
            level_it[0] = SierpinskiTriangle(middle, t.a, t.straight_corner, first_dir, !t.straight_corner_is_left, t.depth + 1);
            level_it[1] = SierpinskiTriangle(middle, t.straight_corner, t.b, second_dir, !t.straight_corner_is_left, t.depth + 1);
        }
    }
    // set totals of leaves
    for (Node max_level_it = pre_division_tree.begin(max_depth); max_level_it != pre_division_tree.end(max_depth); ++max_level_it)
    {
        SierpinskiTriangle& triangle = *max_level_it; 
        float density = density_provider(triangle.a, triangle.b, triangle.straight_corner, triangle.straight_corner);
        float area = 0.5 * INT2MM2(vSize2(triangle.a - triangle.straight_corner));
        triangle.total_value = density * area;
    }
    // bubble total up
    for (int depth = max_depth - 1; depth >= 0; depth--)
    {
        for (Node level_it = pre_division_tree.begin(max_depth); level_it != pre_division_tree.end(max_depth); ++level_it)
        {
            for (SierpinskiTriangle& child : level_it)
            {
                level_it->total_value += child.total_value;
            }
        }
    }
    
    for (int it = 1; it < max_depth; it++)
    {
        process(it);
    }
}

SierpinskiFill::~SierpinskiFill()
{
}

size_t SierpinskiFill::calculatePreDivisionTreeSize(unsigned int max_depth)
{
    constexpr unsigned int child_count = 2;
    unsigned int size = 0;
    unsigned int level_size = 1;
    for (unsigned int depth = 0; depth < max_depth; depth++)
    {
        size += level_size;
        level_size *= child_count;
    }
    return size;
}

void SierpinskiFill::debugOutput(SVG& svg)
{
    svg.writePolygon(aabb.toPolygon(), SVG::Color::RED);
    int i = 0;
    for (SierpinskiFillEdge& edge : edges)
    {
        Point new_l = edge.l + normal(edge.r - edge.l, 150);
        Point new_r = edge.r - normal(edge.r - edge.l, 150);
        svg.writeLine(new_l, new_r);
        svg.writePoint(new_l, straight, 3);
//         svg.writeText((edge.l + edge.r) / 2, std::to_string(i));
        svg.writeText((edge.l + edge.r) / 2, std::to_string(edge.depth));
        i++;
    }
}

void SierpinskiFill::process(int iteration)
{
    const std::function<bool (SierpinskiFillEdge&, SierpinskiFillEdge&)> recurse_triangle = [this] (SierpinskiFillEdge& e1, SierpinskiFillEdge& e2)
        {
            float supposed_density = density_provider(e1.l, e1.r, e2.l, e2.r);
            
            const coord_t average_length = vSize((e1.l + e1.r) - (e2.l + e2.r)) / 2;
            // calculate area of triangle: base times height * .5
            coord_t area;
            {
                const coord_t base_length = vSize(e1.l - e1.r);
                const Point v1 = e1.r - e1.l;
                const Point v2 = e2.r - e2.l;
                const Point height_vector = dot(v2, turn90CCW(v1)) / vSize(v1);
                const coord_t height = vSize(height_vector);
                area = base_length * height / 2;
            }
            assert(area > 0);
            return supposed_density * area > average_length * line_width;
        };

    bool processing_direction = iteration % 2 == 1;
    const bool opposite_direction = !processing_direction;
    using iter = std::list<SierpinskiFillEdge>::iterator;
    SierpinskiFillEdge* prev = nullptr;
    for (iter it = edges.begin(); it != edges.end(); ++it)
    {
        SierpinskiFillEdge& here = *it;
        iter next_it = it;
        ++next_it;
        SierpinskiFillEdge* next = nullptr;
        if (next_it != edges.end())
        {
            next = &*next_it;
        }

        if ((!prev || (prev->direction == opposite_direction && recurse_triangle(*prev, here)))
            && here.direction == processing_direction
            && (!next || (next->direction == opposite_direction && recurse_triangle(here, *next)))
            && (!prev || prev-> depth == iteration - 1) && (!next || here.depth == iteration - 1) // only subdivide triangles of previous iteration
        )
        { // SDS -> SDDDS or DSD -> DSSSD                                    .
            // D step:                                                       .
            // |      /       |\     /                                       .
            // |    /         |  \ /                                         .
            // |  /           | .  \                                         .
            // |/_______  ==> |._____\_                                      .
            //                                                               .
            //                                                               .
            // S step:                                                       .
            //      /                 /|                                     .
            //    /                 /  |                                     .
            //  /__________  ==>  /....|_____                                .
            //  \                 \    |                                     .
            //    \                 \  |                                     .
            //      \                 \|                                     .
            Point mid = (here.l + here.r) / 2;
            if (processing_direction == diagonal)
            {
                here.l = mid;
            }
            else
            {
                here.r = mid;
            }
            here.depth = iteration;

            const Point lr = here.r - here.l;
            const Point lr_T = turn90CCW(lr);
            if (prev)
            {
                prev->depth = iteration; // first trignle is subdivided

                Point l = mid;
                Point r = mid - lr_T;
                if (processing_direction == straight)
                {
                    std::swap(l, r);
                }
                edges.emplace(it, processing_direction, l, r, here.depth);
            }
            if (next)
            {
                Point l = mid;
                Point r = mid + lr_T;
                if (processing_direction == straight)
                {
                    std::swap(l, r);
                }
                edges.emplace(next_it, processing_direction, l, r, here.depth);

                // skip the just created edge
                prev = next;
                ++it;
                ++it;
                continue;
            }
        }
        else if (prev && prev->direction == opposite_direction && here.direction == opposite_direction && recurse_triangle(*prev, here)
            && prev->depth == iteration - 1 // only subdivide triangles of previous iteration
        )
        { // SS -> SDS or DD -> DSD                                  .
            // D step:                                               .
            // |           |    /                                    .
            // |           |  /                                      .
            // |______ ==> |/_____                                   .
            //                                                       .
            // S step:                                               .
            //     /            /                                    .
            //   /            /                                      .
            // /       ==>  /_____                                   .
            // \            \                                        .
            //   \            \                                      .
            //     \            \                                    .
            const Point l = (prev->l + here.l) / 2;
            const Point r = (prev->r + here.r) / 2;
            prev->depth = iteration;
            edges.emplace(it, processing_direction, l, r, prev->depth);
        }

        prev = &here;
    }
}


Polygon SierpinskiFill::generateCross() const
{
    Polygon ret;
    for (const SierpinskiFillEdge& e : edges)
    {
        ret.add((e.l + e.r) / 2);
    }
    return ret;
}
Polygon SierpinskiFill::generateSierpinski() const
{
    Polygon ret;

    
    using Node = FingerTree<SierpinskiTriangle, 2>::Node;
    
    FingerTree<SierpinskiTriangle, 2>& pre_division_tree = const_cast<FingerTree<SierpinskiTriangle, 2>&>(this->pre_division_tree);
    
    for (Node max_level_it = pre_division_tree.begin(max_depth); max_level_it != pre_division_tree.end(max_depth); ++max_level_it)
    {
        SierpinskiTriangle& triangle = *max_level_it; 
        ret.add((triangle.a + triangle.b + triangle.straight_corner) / 3);
    }
    
    return ret;
}


Polygon SierpinskiFill::generateCross(coord_t z, coord_t min_dist_to_side) const
{
    Polygon ret;
    int last_nonD_depth = edges.front().depth;
    for (const SierpinskiFillEdge& e : edges)
    {
        int depth = e.depth;
        if (e.direction == diagonal)
        {
            depth = last_nonD_depth;
        }
        else
        {
            last_nonD_depth = depth;
        }
        
        coord_t period =  8 << (14 - depth / 2);
        coord_t from_l = z % (period * 2);
        if (from_l > period)
        {
            from_l = period * 2 - from_l;
        }
        from_l = from_l * vSize(e.l - e.r) / period;
        from_l = std::max(min_dist_to_side, from_l);
        from_l = std::min(vSize(e.l - e.r) - min_dist_to_side, from_l);
//         if (e.direction == diagonal)
//         {
//             from_l = sqrt(from_l * from_l * 2) / 2;
//         }
        ret.add(e.l + normal(e.r - e.l, from_l));
    }
    return ret;
}


}; // namespace cura
