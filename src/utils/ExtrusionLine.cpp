//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>

#include "ExtrusionLine.h"
#include "linearAlg2D.h"
#include "polygonUtils.h"
#include "Simplify.h"

namespace cura
{

ExtrusionLine::ExtrusionLine(const size_t inset_idx, const bool is_odd)
: inset_idx(inset_idx)
, is_odd(is_odd)
, is_closed(false)
{}

coord_t ExtrusionLine::getLength() const
{
    if (junctions.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions.front();
    for (const ExtrusionJunction& next : junctions)
    {
        len += vSize(next.p - prev.p);
        prev = next;
    }
    if (is_closed)
    {
        len += vSize(front().p - back().p);
    }
    return len;
}

coord_t ExtrusionLine::getMinimalWidth() const
{
    return std::min_element(junctions.cbegin(), junctions.cend(),
                            [](const ExtrusionJunction& l, const ExtrusionJunction& r)
                            {
                                return l.w < r.w;
                            })->w;
}

void ExtrusionLine::cutPolyline(const Polygons& tool, VariableWidthLines* p_results) const
{
    // Rather than roll our own, with all risks that entails, hammer the data in shape until clipper can be used.
    // Since all other properties should be shared with the ExtrusionLine, it's just a matter of re-applying the weight.

    if (junctions.empty())
    {
        return;
    }

    VariableWidthLines& results = *p_results;

    // Initialize lookup for the weights, and stuff the junctions into a polygon (losing information now stored in the map).
    std::map<std::pair<coord_t, coord_t>, coord_t> pos_to_weight;
    Polygons before_clip;
    before_clip.emplace_back();
    for (const auto& junction : junctions)
    {
        pos_to_weight[{junction.p.X, junction.p.Y}] = junction.w;
        before_clip.back().add(junction.p);
    }
    if (is_closed)
    {
        before_clip.back().add(junctions[0].p);
    }

    // Clip the derived 'polygon'.
    const Polygons clipped = tool.intersectionPolyLines(before_clip);

    // Add the clipped paths to the results, (re)find correct weights along the process.
    for (const auto& path : clipped)
    {
        // Start a new path in the results.
        results.emplace_back();
        auto& current_result = results.back();
        current_result.inset_idx = inset_idx;
        current_result.is_odd = is_odd;
        current_result.is_closed = false;

        // Add current connected path to the results.
        for (const Point p : path)
        {
            const auto& weight_idx = pos_to_weight.find({p.X, p.Y});
            if (weight_idx != pos_to_weight.end())
            {
                // Point was in the original junctions-list, so add the junction back with it's original weight.
                current_result.emplace_back(p, weight_idx->second, inset_idx);
                continue;
            }

            // Point is the result of clipping, so find the 'weighted weigth' it would have on the line segment it originally had, then add that.
            const auto closest = PolygonUtils::findClosest(p, before_clip);
            const Point& a = closest.p();
            const Point& b = before_clip[closest.poly_idx][(closest.point_idx + 1) % before_clip[closest.poly_idx].size()];
            const auto& weight_a_idx = pos_to_weight.find({a.X, a.Y});
            const auto& weight_b_idx = pos_to_weight.find({b.X, b.Y});
            if (weight_a_idx == pos_to_weight.end() || weight_b_idx == pos_to_weight.end())
            {
                continue; // TODO?: This actually shouldn't happen, but it seems there may be an edge case that was overlooked.
            }
            const coord_t len_pa = vSize(p - a);
            const coord_t len_ba = vSize(b - a);
            const coord_t w = ((weight_a_idx->second * len_pa) + (weight_b_idx->second * (len_ba - len_pa))) / len_ba;
            current_result.emplace_back(p, w, inset_idx);
        }
    }
}

} // end namespace cura
