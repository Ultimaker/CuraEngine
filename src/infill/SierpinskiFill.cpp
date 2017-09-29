/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SierpinskiFill.h"

#include <algorithm> // swap 

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
        process(it % 2);
    }
}

void SierpinskiFill::debugOutput(SVG& svg)
{
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

void SierpinskiFill::process(bool processing_direction)
{
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

        if ((!prev || prev->direction == opposite_direction)
            && here.direction == processing_direction
            && (!next || next->direction == opposite_direction))
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
        else if (prev && prev->direction == opposite_direction && here.direction == opposite_direction)
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
            assert (e.r == prev->r);
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
