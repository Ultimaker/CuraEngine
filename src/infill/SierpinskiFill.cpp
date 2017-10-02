/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SierpinskiFill.h"

#include <algorithm> // swap 
#include <functional> // function

#include "../utils/linearAlg2D.h" // rotateAround

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

SierpinskiFill::SierpinskiFill(const AABB aabb, int max_depth)
: aabb(aabb)
{
    Point m = aabb.min;
    edges.emplace_back(straight, m, Point(aabb.max.X, m.Y), 2);
    edges.emplace_back(diagonal, m, aabb.max, 1);
    edges.emplace_back(straight, m, Point(m.X, aabb.max.Y), 2);
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

void SierpinskiFill::debugOutput(SVG& svg)
{
    svg.writePolygon(aabb.toPolygon(), SVG::Color::RED);
    int i = 0;
    for (Edge& edge : edges)
    {
        Point new_l = edge.l + normal(edge.r - edge.l, 15);
        Point new_r = edge.r - normal(edge.r - edge.l, 15);
        svg.writeLine(new_l, new_r);
        svg.writePoint(new_l, straight, 3);
        svg.writeText((edge.l + edge.r) / 2, std::to_string(i));
//         svg.writeText((edge.l + edge.r) / 2, std::to_string(edge.depth));
        i++;
    }
}

void SierpinskiFill::process(int iteration)
{
    bool processing_direction = iteration % 2 == 1;
    float prev_density = 1.25 / 512.0 * sqrt(double(1 << (iteration - 1)));
    float density = 1.25 / 512.0 * sqrt(double(1 << iteration));
    std::function<bool (const Edge& e1, const Edge e2)> recurse_triangle =
    [processing_direction, density, prev_density, iteration, this](const Edge& e1, const Edge e2)->bool
        {
            int depth_diff = e2.depth - e1.depth;
            if (e1.direction == e2.direction)
            {
                if (e1.depth != e2.depth)
                {
                    return false;
                }
            }
            else
            {
                if (depth_diff < -1 || depth_diff > 1)
                {
                    return false;
                }
            }
            AABB aabb_here;
            aabb_here.include(e1.l);
            aabb_here.include(e1.r);
            aabb_here.include(e2.l);
            aabb_here.include(e2.r);
            Point min = (aabb_here.min - aabb.min - Point(1,1)) * pic_size.X / (aabb.max - aabb.min);
            Point max = (aabb_here.max - aabb.min + Point(1,1)) * pic_size.Y / (aabb.max - aabb.min);
            long tot = 0;
            int pixel_count = 0;
            for (int x = std::max((coord_t)0, min.X); x <= std::min((coord_t)pic_size.X - 1, max.X); x++)
            {
                for (int y = std::max((coord_t)0, min.Y); y <= std::min((coord_t)pic_size.Y - 1, max.Y); y++)
                {
                    tot += pic[pic_size.Y - 1 - y][x];
                    pixel_count++;
                }
            }
            long boundary = (1.0 - (density + prev_density) * .5) * 255 * pixel_count;
            if (tot < boundary)
            { 
                return true;
            }
            return false;
        };


    const bool opposite_direction = !processing_direction;
    using iter = std::list<Edge>::iterator;
    Edge* prev = nullptr;
    for (iter it = edges.begin(); it != edges.end(); ++it)
    {
        Edge& here = *it;
        iter next_it = it;
        ++next_it;
        Edge* next = nullptr;
        if (next_it != edges.end())
        {
            next = &*next_it;
        }

        if ((!prev || (prev->direction == opposite_direction && recurse_triangle(*prev, here)))
            && here.direction == processing_direction
            && (!next || (next->direction == opposite_direction && recurse_triangle(here, *next)))
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
            here.depth += 2;

            const Point lr = here.r - here.l;
            const Point lr_T = turn90CCW(lr);
            if (prev)
            {
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
            }
        }
        else if (prev && prev->direction == opposite_direction && here.direction == opposite_direction && recurse_triangle(*prev, here))
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
            edges.emplace(it, processing_direction, l, r, here.depth + 1);
        }

        prev = &here;
    }
}


Polygon SierpinskiFill::generateCross() const
{
    Polygon ret;
    for (const Edge& e : edges)
    {
        ret.add((e.l + e.r) / 2);
    }
    return ret;
}
Polygon SierpinskiFill::generateSierpinski() const
{
    Polygon ret;

    const Edge* prev = &edges.back();
    for (const Edge& e : edges)
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

}; // namespace cura
