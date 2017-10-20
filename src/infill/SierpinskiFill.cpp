/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SierpinskiFill.h"

#include <algorithm> // swap 
#include <functional> // function

#include "../utils/linearAlg2D.h" // rotateAround

#include "ImageBasedSubdivider.h"
#include "UniformSubdivider.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

SierpinskiFill::SierpinskiFill(const Subdivider& subdivider, const AABB aabb, int max_depth)
: subdivider(subdivider)
, aabb(aabb)
{
    Point m = aabb.min;
    edges.emplace_back(straight, m, Point(aabb.max.X, m.Y), 0);
    edges.emplace_back(diagonal, m, aabb.max, 0);
    edges.emplace_back(straight, m, Point(m.X, aabb.max.Y), 0);
    /*
     *    Point m = aabb.getMiddle();
     *    edges.emplace_back(straight, m, Point(aabb.max.X, m.Y));
     *    edges.emplace_back(diagonal, m, aabb.max);
     *    edges.emplace_back(straight, m, Point(m.X, aabb.max.Y));
     *    edges.emplace_back(diagonal, m, Point(aabb.min.X, aabb.max.Y));
     *    edges.emplace_back(straight, m, Point(aabb.min.X, m.Y));
     *    edges.emplace_back(diagonal, m, Point(aabb.min.X, aabb.min.Y));
     *    edges.emplace_back(straight, m, Point(m.X, aabb.min.Y));
     *    edges.emplace_back(diagonal, m, Point(aabb.max.X, aabb.min.Y));
     *    edges.emplace_back(straight, m, Point(aabb.max.X, m.Y));
     */

    for (int it = 1; it < max_depth; it++)
    {
        process(it);
    }
}

SierpinskiFill::~SierpinskiFill()
{
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
    const Subdivider& recurse_triangle = subdivider;

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

    const SierpinskiFillEdge* prev = &edges.back();
    for (const SierpinskiFillEdge& e : edges)
    {
        Point c;
        if (e.l == prev->l)
        {
            c = prev->r;
        }
        else
        {
//             assert (e.r == prev->r);
            c = prev->l;
        }
        Point tot = e.l + e.r + c;
        Point add;
        coord_t lr_size2 = vSize2(e.r - e.l);
        coord_t lc_size2 = vSize2(c - e.l);
        coord_t rc_size2 = vSize2(c - e.r);
        if (lr_size2 > lc_size2 + 10 && lr_size2 > rc_size2 + 10)
        {
            add = c;
        }
        else if (lc_size2 > lr_size2 + 10 && lc_size2 > rc_size2 + 10)
        {
            add = e.r;
        }
        else
        {
            add = e.l;
        }
        tot += add;
        ret.add(tot / 4);
        prev = &e;
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
