//Copyright (c) 2019 Ultimaker B.V.


#include "BeadingOrderOptimizer.h"

#include <unordered_map>
#include <vector>
#include <list>

namespace arachne
{


void BeadingOrderOptimizer::optimize(const std::vector<ExtrusionSegment>& segments, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polylines_per_index)
{
    struct Polyline
    {
        coord_t inset_idx;
        std::list<ExtrusionJunction> junctions;
        Polyline(coord_t inset_idx)
        : inset_idx(inset_idx)
        {}
    };
    
    struct PolylineEndRef
    {
        coord_t inset_idx;
        std::list<Polyline>::iterator polyline;
        bool front;
        PolylineEndRef(coord_t inset_idx, std::list<Polyline>::iterator polyline, bool front)
        : inset_idx(inset_idx)
        , polyline(polyline)
        , front(front)
        {}
    };

    std::list<Polyline> polylines;
    std::unordered_map<Point, PolylineEndRef> polyline_end_points(segments.size());

    auto debugCheck = [&polylines, &polyline_end_points, &segments]()
        {
#ifdef DEBUG
            for (Polyline& polyline : polylines)
            {
                for (ExtrusionJunction& junction : polyline.junctions)
                {
                    assert(junction.perimeter_index == polyline.inset_idx);
                    assert(junction.p.X < 100000 && junction.p.Y < 100000);
                    assert(junction.p.X > -100000 && junction.p.Y > -100000);
                }
            }
            for (auto pair : polyline_end_points)
            {
                PolylineEndRef ref = pair.second;
                Point p = pair.first;
                assert(ref.inset_idx < polylines.size());
                assert(p == ((ref.front)? ref.polyline->junctions.front().p : ref.polyline->junctions.back().p));
                auto find_it = polylines.begin();
                for (; find_it != polylines.end(); ++find_it)
                {
                    if (find_it == ref.polyline) break;
                }
                assert(find_it != polylines.end());
            }
            for (Polyline& polyline : polylines)
            {
                auto prev_it = polyline.junctions.begin();
                for (auto junction_it = prev_it; junction_it != polyline.junctions.end();)
                {
                    ++junction_it;
                    if (junction_it == polyline.junctions.end()) break;
                    
                    ExtrusionJunction& prev = *prev_it;
                    ExtrusionJunction& next = *junction_it;
                    
                    bool found = false;
                    for (const ExtrusionSegment& segment : segments)
                    {
                        if ((segment.from == prev && segment.to == next)
                            || (segment.to == prev && segment.from == next))
                        {
                            found = true;
                            break;
                        }
                    }
                    assert(found);
                    prev_it = junction_it;
                }
            }
#endif
        };


    debugCheck();
    for (const ExtrusionSegment& segment : segments)
    {
        if (segment.from.p == segment.to.p)
        {
            assert(segment.from.w == segment.to.w);
            continue;
        }

        assert(segment.from.perimeter_index == segment.to.perimeter_index);
        coord_t inset_idx = segment.from.perimeter_index;

        auto start_it = polyline_end_points.find(segment.from.p);
        auto end_it = polyline_end_points.find(segment.to.p);

        bool connect_start = start_it != polyline_end_points.end();
        bool connect_end = end_it != polyline_end_points.end();
        debugCheck();
        if (!connect_start && !connect_end)
        {
            polylines.emplace_back(inset_idx);
            std::list<ExtrusionJunction>& junctions = polylines.back().junctions;
            junctions.emplace_front(segment.from);
            junctions.emplace_back(segment.to);
            polyline_end_points.emplace(segment.from.p, PolylineEndRef(inset_idx, --polylines.end(), true));
            polyline_end_points.emplace(segment.to.p, PolylineEndRef(inset_idx, --polylines.end(), false));
        }
        else if (connect_start && !connect_end)
        {
            PolylineEndRef ref = start_it->second;
            if (ref.front)
            {
                ref.polyline->junctions.emplace_front(segment.to);
            }
            else
            {
                ref.polyline->junctions.emplace_back(segment.to);
            }
            polyline_end_points.erase(start_it);
            polyline_end_points.emplace(segment.to.p, ref);
        }
        else if (!connect_start && connect_end)
        {
            PolylineEndRef ref = end_it->second;
            if (ref.front)
            {
                ref.polyline->junctions.emplace_front(segment.from);
            }
            else
            {
                ref.polyline->junctions.emplace_back(segment.from);
            }
            polyline_end_points.erase(end_it);
            polyline_end_points.emplace(segment.from.p, ref);
        }
        else // if (connect_start && connect_end)
        {
            PolylineEndRef start_ref = start_it->second;
            PolylineEndRef end_ref = end_it->second;

            std::list<ExtrusionJunction>& result_junctions = start_ref.polyline->junctions;

            assert(start_it != end_it);
            polyline_end_points.erase(start_it);
            polyline_end_points.erase(end_it);
            if (&*start_ref.polyline != &*end_ref.polyline)
            {
                polyline_end_points.erase((start_ref.front)? start_ref.polyline->junctions.back().p : start_ref.polyline->junctions.front().p);
                polyline_end_points.erase((end_ref.front)? end_ref.polyline->junctions.back().p : end_ref.polyline->junctions.front().p);
                std::list<ExtrusionJunction>::iterator insert_position = start_ref.front? result_junctions.begin() : result_junctions.end();
                if (end_ref.front == start_ref.front)
                {
                    result_junctions.insert(insert_position, end_ref.polyline->junctions.rbegin(), end_ref.polyline->junctions.rend());
                }
                else
                {
                    result_junctions.insert(insert_position, end_ref.polyline->junctions.begin(), end_ref.polyline->junctions.end());
                }
                polyline_end_points.emplace(result_junctions.front().p, PolylineEndRef(start_ref.inset_idx, start_ref.polyline, true));
                polyline_end_points.emplace(result_junctions.back().p, PolylineEndRef(start_ref.inset_idx, start_ref.polyline, false));
                polylines.erase(end_ref.polyline);
            }
            else // if start_ref.polyline == end_ref.polyline
            {
                polyline_end_points.erase((start_ref.front)? start_ref.polyline->junctions.back().p : start_ref.polyline->junctions.front().p);
                polyline_end_points.erase((end_ref.front)? end_ref.polyline->junctions.back().p : end_ref.polyline->junctions.front().p);
                // close polygon and add it to the results
                result_polygons_per_index.resize(std::max(result_polygons_per_index.size(), static_cast<size_t>(start_ref.inset_idx + 1)));
                result_polygons_per_index[start_ref.inset_idx].emplace_back(result_junctions.begin(), result_junctions.end());
                polylines.erase(start_ref.polyline);
            }
        }
    }
    debugCheck();

    // copy unfinished polylines to result
    result_polylines_per_index.resize(std::max(result_polylines_per_index.size(), polylines.size()));
    for (Polyline& polyline : polylines)
    {
        result_polylines_per_index[polyline.inset_idx].emplace_back(polyline.junctions.begin(), polyline.junctions.end());
    }
}

} // namespace arachne
