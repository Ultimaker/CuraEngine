//Copyright (c) 2020 Ultimaker B.V.


#include "BeadingOrderOptimizer.h"

#include <iterator>
#include <unordered_map>
#include <vector>
#include <list>

#include "utils/SparsePointGridInclusive.h"

namespace arachne
{

void BeadingOrderOptimizer::optimize(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool reduce_overlapping_segments, bool connect_odd_lines_to_polygons)
{
    BeadingOrderOptimizer optimizer(polylines_per_index);
    optimizer.fuzzyConnect(polygons_per_index, snap_dist, reduce_overlapping_segments, connect_odd_lines_to_polygons);
}

BeadingOrderOptimizer::BeadingOrderOptimizer(std::vector<std::list<ExtrusionLine>>& polylines_per_index)
: polylines_per_index(polylines_per_index)
{
    for (auto& polylines : polylines_per_index)
    {
        for (std::list<ExtrusionLine>::iterator polyline_it = polylines.begin(); polyline_it != polylines.end(); ++polyline_it)
        {
            ExtrusionLine& polyline = *polyline_it;
            assert(polyline.junctions.size() >= 2); // Otherwise the front and back ExtrusionLineEndRef would be mapped to from the same location
            if (polyline.junctions.empty()) 
            {
                continue; // Shouldn't happen 
            }
            polyline_end_points.emplace(polyline.junctions.front().p, ExtrusionLineEndRef(polyline.inset_idx, polyline_it, true));
            polyline_end_points.emplace(polyline.junctions.back().p, ExtrusionLineEndRef(polyline.inset_idx, polyline_it, false));
        }
    }
}


void BeadingOrderOptimizer::fuzzyConnect(std::vector<std::list<ExtrusionLine>>& polygons_per_index, coord_t snap_dist, bool reduce_overlapping_segments, bool connect_odd_lines_to_polygons)
{
    struct Locator
    {
        Point operator()(const ExtrusionLineEndRef& it)
        {
            return it.p();
        }
    };
    SparsePointGridInclusive<ExtrusionLineEndRef> polyline_grid(snap_dist); // Inclusive because iterators might get invalidated
    for (std::list<ExtrusionLine>& polys : polylines_per_index)
    {
        for (auto poly_it = polys.begin(); poly_it != polys.end(); ++poly_it)
        {
            for (bool front : { true, false })
            {
                ExtrusionLineEndRef ref(poly_it->inset_idx, poly_it, front);
                polyline_grid.insert(ref.p(), ref);
            }
        }
    }
    
    struct PointLocator
    {
        Point operator()(const Point& p) 
        { 
            return p; 
        }
    };
    SparsePointGrid<Point, PointLocator> polygon_grid(snap_dist); // Inclusive because iterators might get invalidated
    for (auto polygons : polygons_per_index)
    {
        for (auto polygon : polygons)
        {
            for (ExtrusionJunction& junction : polygon.junctions)
            {
                polygon_grid.insert(junction.p);
            }
        }
    }

    // Order of end_points_to_check and nearby_end_points determines which ends are connected together
    // TODO: decide on the best way to connect polylines at 3-way intersections
    std::list<ExtrusionLineEndRef> end_points_to_check;

    for (std::list<ExtrusionLine>& polys : polylines_per_index)
    {
        for (bool odd_lines : { connect_odd_lines_to_polygons, !connect_odd_lines_to_polygons })
        { // Try to combine odd lines into polygons first, if connect_odd_lines_to_polygons
            for (auto poly_it = polys.begin(); poly_it != polys.end(); ++poly_it)
            {
                if (poly_it->is_odd != odd_lines)
                {
                    continue;
                }
                
                assert(poly_it->junctions.size() > 1);
                for (bool front : { true, false })
                {
                    end_points_to_check.emplace_back(poly_it->inset_idx, poly_it, front);
                }
            }
        }
    }
    
    for (auto it = end_points_to_check.begin(); it != end_points_to_check.end(); ++it)
    {
        ExtrusionLineEndRef& end_point = *it;
        if (end_point.polyline->junctions.empty())
        { // There is no line. We cannot do anything because we don't know where this ghost line initially was
            continue;
        }

        Point p = end_point.p();

        bool end_point_has_changed = false; // Whether the end_point ref now points to a new end compared to where it was initially pointing to
        bool has_connected = polygon_grid.getAnyNearby(p, snap_dist) != nullptr; // We check whether polygons were already connected together here

        if (has_connected && !reduce_overlapping_segments)
        { // Don't process nearby polyline endpoints
            continue;
        }
        if (has_connected)
        {
            coord_t extrusion_width = end_point.front? end_point.polyline->junctions.front().w : end_point.polyline->junctions.back().w;
            if (end_point.front)
            {
                reduceIntersectionOverlap(*end_point.polyline, end_point.polyline->junctions.begin(), 0, extrusion_width / 2, end_point);
            }
            else
            {
                reduceIntersectionOverlap(*end_point.polyline, end_point.polyline->junctions.rbegin(), 0, extrusion_width / 2, end_point);
            }
        }

        std::vector<ExtrusionLineEndRef> nearby_end_points = polyline_grid.getNearbyVals(p, snap_dist);
        for (size_t other_end_idx = 0; other_end_idx < nearby_end_points.size(); ++other_end_idx)
        {
            ExtrusionLineEndRef& other_end = nearby_end_points[other_end_idx];
            assert(!end_point.polyline->junctions.empty() || has_connected);
            if (other_end.polyline->junctions.empty())
            {
                continue;
            }
            
            if (end_point == other_end && (!has_connected || !end_point_has_changed))
            {
                // Only reduce overlap if the end point ref hasn't changed because of connecting
                // NOTE: connecting might change the junctions in polyline and therefore the ExtrusionLineRef [end_point] might now refer to a new end point!
                continue;
            }

            if (!shorterThen(p - other_end.p(), snap_dist))
            { // The other end is not really a nearby other end
                continue;
            }
            
            coord_t other_end_polyline_length = other_end.polyline->computeLength();
            if (&*end_point.polyline == &*other_end.polyline
                && (other_end.polyline->junctions.size() <= 2 || other_end_polyline_length < snap_dist * 2))
            { // The other end is of the same really short polyline
                continue;
            }

            if (!has_connected)
            {
                int_fast8_t changed_side_is_front = -1; // Unset bool; whether we have changed what the front of the polyline of [end_point] is rather than the back (stays -1 if we don't need to reinsert any list change because it is now a closed polygon)
                if (&*end_point.polyline == &*other_end.polyline)
                { // We can close this polyline into a polygon
                    polygons_per_index.resize(std::max(polygons_per_index.size(), static_cast<size_t>(end_point.inset_idx + 1)));
                    polygons_per_index[end_point.inset_idx].emplace_back(*end_point.polyline);
                    for (ExtrusionJunction& junction : end_point.polyline->junctions)
                    {
                        polygon_grid.insert(junction.p);
                    }
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
                    ExtrusionLineEndRef line_untouched_end(end_point.inset_idx, end_point.polyline, changed_side_is_front);
                    polyline_grid.insert(line_untouched_end.p(), line_untouched_end);
                    end_points_to_check.emplace_back(line_untouched_end);
                    nearby_end_points.emplace_back(line_untouched_end);
                    end_point_has_changed = end_point.front == changed_side_is_front && other_end_polyline_length > snap_dist * 2;
                }
                has_connected = true;
                if (!reduce_overlapping_segments)
                { // Don't continue going over nearby polyline ends
                    break;
                }
            }
            else
            {
                coord_t extrusion_width = other_end.front? other_end.polyline->junctions.front().w : other_end.polyline->junctions.back().w;
                if (other_end.front)
                {
                    reduceIntersectionOverlap(*other_end.polyline, other_end.polyline->junctions.begin(), 0, extrusion_width / 2, end_point);
                }
                else
                {
                    reduceIntersectionOverlap(*other_end.polyline, other_end.polyline->junctions.rbegin(), 0, extrusion_width / 2, end_point);
                }
            }
        }
    }

    // Remove emptied lists
    // This is done afterwards, because otherwise iterators would be invalid during the connecting algorithm
    for (std::list<ExtrusionLine>& polys : polylines_per_index)
    {
        for (auto poly_it = polys.begin(); poly_it != polys.end();)
        {
            if (poly_it->junctions.empty() || poly_it->computeLength() < 2 * snap_dist) 
            // Too small segments might have been overlooked byecause of the fuzzy nature of matching end points to each other
            {
                poly_it = polys.erase(poly_it);
            }
            else
            {
                poly_it++;
            }
        }
    }
}

template<typename directional_iterator>
void BeadingOrderOptimizer::reduceIntersectionOverlap(ExtrusionLine& polyline, directional_iterator polyline_it, coord_t traveled_dist, coord_t reduction_length, ExtrusionLineEndRef& reduction_source)
{
    ExtrusionJunction& start_junction = *polyline_it;
    directional_iterator next_junction_it = polyline_it; next_junction_it++;
    if (isEnd(next_junction_it, polyline))
    {
        return;
    }
    
    if (&*reduction_source.polyline == &polyline && polyline.junctions.size() <= 2)
    { // Don't throw away small segments overlapping with themselves
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
        reduceIntersectionOverlap(polyline, next_junction_it, traveled_dist + length, reduction_length, reduction_source);
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
bool BeadingOrderOptimizer::isEnd(std::list<ExtrusionJunction>::iterator it, ExtrusionLine& polyline)
{
    return it == polyline.junctions.end();
}

template<>
bool BeadingOrderOptimizer::isEnd(std::list<ExtrusionJunction>::reverse_iterator it, ExtrusionLine& polyline)
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

void BeadingOrderOptimizer::debugCheck()
{
#ifdef DEBUG
    for (auto& polylines : polylines_per_index)
    {
        for (ExtrusionLine& polyline : polylines)
        {
            for (ExtrusionJunction& junction : polyline.junctions)
            {
                assert(junction.perimeter_index == polyline.inset_idx);
                assert(junction.p.X < 1000000 && junction.p.Y < 1000000);
                assert(junction.p.X > -1000000 && junction.p.Y > -1000000);
            }
        }
    }
#endif
};
} // namespace arachne
