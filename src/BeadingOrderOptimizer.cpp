//Copyright (c) 2019 Ultimaker B.V.


#include "BeadingOrderOptimizer.h"

#include <iterator>
#include <unordered_map>
#include <vector>
#include <list>

#include "utils/SparsePointGridInclusive.h"

namespace arachne
{

void BeadingOrderOptimizer::optimize(const std::vector<ExtrusionSegment>& segments, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polylines_per_index)
{
    BeadingOrderOptimizer optimizer(segments);
    optimizer.connect(result_polygons_per_index);
    optimizer.fuzzyConnect(result_polygons_per_index, snap_dist);
    optimizer.transferUnconnectedPolylines(result_polygons_per_index, result_polylines_per_index);
}

void BeadingOrderOptimizer::connect(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index)
{
    struct count // wrapper which initializes to zero in default construction
    { // NOTE: you cannot inherit from primitive types
        coord_t n;
        count() : n(0) { }
    };
    std::unordered_map<Point, count> connection_counts;
    for (const ExtrusionSegment& segment : segments)
    {
        if (segment.from.p != segment.to.p)
        {
            connection_counts[segment.from.p].n++;
            connection_counts[segment.to.p].n++;
        }
    }

    debugCheck();
    for (const ExtrusionSegment& segment : segments)
    {
        if (segment.from.p == segment.to.p)
        {
            assert(std::abs(segment.from.w - segment.to.w) < 5);
            continue;
        }

        assert(segment.from.perimeter_index == segment.to.perimeter_index);
        coord_t inset_idx = segment.from.perimeter_index;
        std::list<Polyline>& polylines = segment.is_odd? odd_polylines : even_polylines;
        std::unordered_map<Point, PolylineEndRef>& polyline_end_points = segment.is_odd? odd_polyline_end_points : even_polyline_end_points;

        auto start_it = polyline_end_points.find(segment.from.p);
        auto end_it = polyline_end_points.find(segment.to.p);

        bool connect_start = start_it != polyline_end_points.end();
        bool connect_end = end_it != polyline_end_points.end();

        // don't connect polylines at intersections in this initial connection function
        connect_start &= !segment.is_odd || connection_counts[segment.from.p].n <= 2;
        connect_end &= !segment.is_odd || connection_counts[segment.to.p].n <= 2;

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
}

void BeadingOrderOptimizer::fuzzyConnect(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, coord_t snap_dist)
{
    struct Locator
    {
        Point operator()(const PolylineEndRef& it)
        {
            return it.p();
        }
    };
    SparsePointGridInclusive<PolylineEndRef> polyline_grid(snap_dist); // inclusive because iterators might get invalidated
    for (std::list<Polyline>* polys : { &odd_polylines, &even_polylines })
    {
        for (auto poly_it = polys->begin(); poly_it != polys->end(); ++poly_it)
        {
            for (bool front : { true, false })
            {
                PolylineEndRef ref(poly_it->inset_idx, poly_it, front);
                polyline_grid.insert(ref.p(), ref);
            }
        }
    }
    struct PointLocator
    {
        Point operator()(const Point& p) { return p; }
    };
    SparsePointGrid<Point, PointLocator> polygon_grid(snap_dist); // inclusive because iterators might get invalidated
    for (auto result_polygons : result_polygons_per_index)
        for (auto result_polygon : result_polygons)
            for (ExtrusionJunction& junction : result_polygon)
                polygon_grid.insert(junction.p);

    std::list<PolylineEndRef> end_points_to_check;

    for (std::list<Polyline>* polys : { &odd_polylines, &even_polylines })
    {
        for (auto poly_it = polys->begin(); poly_it != polys->end(); ++poly_it)
        {
            assert(poly_it->junctions.size() > 1);
            if (poly_it->computeLength() > 10)
            {
                for (bool front : { true, false })
                {
                    end_points_to_check.emplace_back(poly_it->inset_idx, poly_it, front);
                }
            }
        }
    }
    for (auto it = end_points_to_check.begin(); it != end_points_to_check.end(); ++it)
    {
        PolylineEndRef& end_point = *it;
        Point p = end_point.p();

        bool end_point_has_changed = false; // Whether the end_point ref now points to a new end compared to where it was initially pointing to
        bool has_connected = polygon_grid.getAnyNearby(p, snap_dist) != nullptr; // we check whether polygons were already connected together here

        if (has_connected)
        {
            coord_t extrusion_width = end_point.front? end_point.polyline->junctions.front().w : end_point.polyline->junctions.back().w;
            if (end_point.front)
            {
                reduceIntersectionOverlap(*end_point.polyline, end_point.polyline->junctions.begin(), 0, extrusion_width / 2);
            }
            else
            {
                reduceIntersectionOverlap(*end_point.polyline, end_point.polyline->junctions.rbegin(), 0, extrusion_width / 2);
            }
        }

        std::vector<PolylineEndRef> nearby_end_points = polyline_grid.getNearbyVals(p, snap_dist);
        for (size_t other_end_idx = 0; other_end_idx < nearby_end_points.size(); ++other_end_idx)
        {
            PolylineEndRef& other_end = nearby_end_points[other_end_idx];
            assert(!end_point.polyline->junctions.empty() || has_connected);
            if (other_end.polyline->junctions.empty())
            {
                continue;
            }
            if (end_point == other_end)
                if ((!has_connected || !end_point_has_changed) // only reduce overlap if the end point ref hasn't changed because of connecting
                // NOTE: connecting might change the junctions in polyline and therefore the PolylineRef [end_point] might now refer to a new end point!
            )
            {
                continue;
            }
            if (!shorterThen(p - other_end.p(), snap_dist)
            )
            { // the other end is not really a nearby other end
                continue;
            }
            coord_t other_end_polyline_length = other_end.polyline->computeLength();
            if (&*end_point.polyline == &*other_end.polyline
                && (other_end.polyline->junctions.size() <= 2 || other_end_polyline_length < snap_dist * 2)
            )
            { // the other end is of the same really short polyline
                continue;
            }
            
            if (!has_connected)
            {
                int_fast8_t changed_side_is_front = -1; // unset bool; whether we have changed what the front of the polyline of [end_point] is rather than the back (stays -1 if we don't need to reinsert any list change because it is now a closed polygon)
                if (&*end_point.polyline == &*other_end.polyline)
                { // we can close this polyline into a polygon
                    result_polygons_per_index.resize(std::max(result_polygons_per_index.size(), static_cast<size_t>(end_point.inset_idx + 1)));
                    result_polygons_per_index[end_point.inset_idx].emplace_back(end_point.polyline->junctions.begin(), end_point.polyline->junctions.end());
                    for (ExtrusionJunction& junction : end_point.polyline->junctions)
                        polygon_grid.insert(junction.p);
                    end_point.polyline->junctions.clear();
                }
                else if (!end_point.front && other_end.front)
                {
                    end_point.polyline->junctions.splice(end_point.polyline->junctions.end(), other_end.polyline->junctions);
                    changed_side_is_front = 0;
                }
                else if (end_point.front && !other_end.front)
                {
                    end_point.polyline->junctions.splice(end_point.polyline->junctions.begin(), other_end.polyline->junctions);
                    changed_side_is_front = 1;
                }
                else if (end_point.front && other_end.front)
                {
                    end_point.polyline->junctions.insert(end_point.polyline->junctions.begin(), other_end.polyline->junctions.rbegin(), other_end.polyline->junctions.rend());
                    other_end.polyline->junctions.clear();
                    changed_side_is_front = 1;
                }
                else // if (!end_point.front && !other_end.front)
                {
                    end_point.polyline->junctions.insert(end_point.polyline->junctions.end(), other_end.polyline->junctions.rbegin(), other_end.polyline->junctions.rend());
                    other_end.polyline->junctions.clear();
                    changed_side_is_front = 0;
                }
                if (changed_side_is_front != -1)
                {
                    PolylineEndRef result_line_untouched_end(end_point.inset_idx, end_point.polyline, changed_side_is_front);
                    polyline_grid.insert(result_line_untouched_end.p(), result_line_untouched_end);
                    end_points_to_check.emplace_back(result_line_untouched_end);
                    nearby_end_points.emplace_back(result_line_untouched_end);
                    end_point_has_changed = end_point.front == changed_side_is_front && other_end_polyline_length > snap_dist * 2;
                }
                has_connected = true;
            }
            else
            {
                coord_t extrusion_width = other_end.front? other_end.polyline->junctions.front().w : other_end.polyline->junctions.back().w;
                if (other_end.front)
                {
                    reduceIntersectionOverlap(*other_end.polyline, other_end.polyline->junctions.begin(), 0, extrusion_width / 2);
                }
                else
                {
                    reduceIntersectionOverlap(*other_end.polyline, other_end.polyline->junctions.rbegin(), 0, extrusion_width / 2);
                }
            }
        }
    }

    // remove emptied lists
    // afterwards, because otherwise iterators would be invalid during the connecting algorithm
    for (std::list<Polyline>* polys : { &odd_polylines, &even_polylines })
    {
        for (auto poly_it = polys->begin(); poly_it != polys->end();)
        {
            if (poly_it->junctions.empty())
            {
                poly_it = polys->erase(poly_it);
            }
            else
            {
                poly_it++;
            }
        }
    }
}

template<typename directional_iterator>
void BeadingOrderOptimizer::reduceIntersectionOverlap(Polyline& polyline, directional_iterator polyline_it, coord_t traveled_dist, coord_t reduction_length)
{
    ExtrusionJunction& start_junction = *polyline_it;
    directional_iterator next_junction_it = polyline_it; next_junction_it++;
    if (isEnd(next_junction_it, polyline))
    {
        return;
    }
    ExtrusionJunction& next_junction = *next_junction_it;
    Point a = start_junction.p;
    Point b = next_junction.p;
    Point ab = b - a;
    coord_t length = vSize(ab);

    coord_t total_reduction_length = reduction_length + start_junction.w / 2;
    total_reduction_length *= (1.0 - intersection_overlap);

    if (length > 0 && traveled_dist + length > total_reduction_length)
    {
        coord_t reduction_left = total_reduction_length - traveled_dist;
        Point mid = a + ab * std::max(static_cast<coord_t>(0), std::min(length, reduction_left)) / length;
        std::list<ExtrusionJunction>::iterator forward_it = getInsertPosIt(next_junction_it);
        coord_t mid_w = start_junction.w + (next_junction.w - start_junction.w) * reduction_left / length;
        polyline.junctions.insert(forward_it, ExtrusionJunction(mid, mid_w, start_junction.perimeter_index));
    }
    else
    {
        // NOTE: polyline_start_it was already increased
        reduceIntersectionOverlap(polyline, next_junction_it, traveled_dist + length, reduction_length);
    }

    if (polyline.junctions.size() > 1)
    {
        polyline.junctions.erase(getSelfPosIt(polyline_it));
    }
    if (polyline.junctions.size() == 1)
    {
        polyline.junctions.clear();
    }
}


template<>
bool BeadingOrderOptimizer::isEnd(std::list<ExtrusionJunction>::iterator it, Polyline& polyline)
{
    return it == polyline.junctions.end();
}

template<>
bool BeadingOrderOptimizer::isEnd(std::list<ExtrusionJunction>::reverse_iterator it, Polyline& polyline)
{
    return it == polyline.junctions.rend();
}

template<>
std::list<ExtrusionJunction>::iterator BeadingOrderOptimizer::getInsertPosIt(std::list<ExtrusionJunction>::iterator it)
{
    return it;
}

template<>
std::list<ExtrusionJunction>::iterator BeadingOrderOptimizer::getInsertPosIt(std::list<ExtrusionJunction>::reverse_iterator it)
{
    return it.base();
}

template<>
std::list<ExtrusionJunction>::iterator BeadingOrderOptimizer::getSelfPosIt(std::list<ExtrusionJunction>::iterator it)
{
    return it;
}

template<>
std::list<ExtrusionJunction>::iterator BeadingOrderOptimizer::getSelfPosIt(std::list<ExtrusionJunction>::reverse_iterator it)
{
    return (++it).base();
}

void BeadingOrderOptimizer::transferUnconnectedPolylines(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polylines_per_index)
{
    // copy unfinished polylines to result
    result_polylines_per_index.resize(std::max(result_polylines_per_index.size(), even_polylines.size() + odd_polylines.size()));
    for (auto* polylines : { &even_polylines, &odd_polylines })
    {
        for (Polyline& polyline : *polylines)
        {
            result_polylines_per_index.resize(std::max(result_polylines_per_index.size(), static_cast<size_t>(polyline.inset_idx + 1)));
            result_polylines_per_index[polyline.inset_idx].emplace_back(polyline.junctions.begin(), polyline.junctions.end());
        }
    }
}


void BeadingOrderOptimizer::debugCheck()
{
#ifdef DEBUG
    for (auto polylines : { even_polylines, odd_polylines })
    {
        for (Polyline& polyline : polylines)
        {
            for (ExtrusionJunction& junction : polyline.junctions)
            {
                assert(junction.perimeter_index == polyline.inset_idx);
                assert(junction.p.X < 1000000 && junction.p.Y < 1000000);
                assert(junction.p.X > -1000000 && junction.p.Y > -1000000);
            }
        }
    }
    for (auto polyline_end_points : { even_polyline_end_points, odd_polyline_end_points })
    {
        for (auto pair : polyline_end_points)
        {
            PolylineEndRef ref = pair.second;
            Point p = pair.first;
            assert(p == ((ref.front)? ref.polyline->junctions.front().p : ref.polyline->junctions.back().p));
            auto find_it = even_polylines.begin();
            for (; find_it != even_polylines.end(); ++find_it)
            {
                if (find_it == ref.polyline) break;
            }
            if (find_it == even_polylines.end())
            {
                for (find_it = odd_polylines.begin(); find_it != odd_polylines.end(); ++find_it)
                {
                    if (find_it == ref.polyline) break;
                }
            }
            assert(find_it != odd_polylines.end());
        }
    }
#endif
};


} // namespace arachne
