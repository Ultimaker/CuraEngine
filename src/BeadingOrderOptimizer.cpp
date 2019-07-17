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
    optimizer.reduceIntersectionOverlap(result_polygons_per_index);
    optimizer.transferUnconnectedPolylines(result_polygons_per_index, result_polylines_per_index);
}

void BeadingOrderOptimizer::connect(std::vector<std::vector<std::vector<ExtrusionJunction>>>& result_polygons_per_index)
{
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
        std::list<Polyline>& polylines = segment.is_odd? odd_polylines : even_polylines;
        std::unordered_map<Point, PolylineEndRef>& polyline_end_points = segment.is_odd? odd_polyline_end_points : even_polyline_end_points;

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
    SparsePointGridInclusive<PolylineEndRef> grid(snap_dist);
    for (std::list<Polyline>* polys : { &odd_polylines, &even_polylines })
    {
        for (auto poly_it = polys->begin(); poly_it != polys->end(); ++poly_it)
        {
            for (bool front : { true, false })
            {
                PolylineEndRef ref(poly_it->inset_idx, poly_it, front);
                grid.insert(ref.p(), ref);
            }
        }
    }

    for (std::list<Polyline>* polys : { &odd_polylines, &even_polylines })
    {
        for (auto poly_it = polys->begin(); poly_it != polys->end(); ++poly_it)
        {
            for (bool front : { true, false })
            {
                PolylineEndRef end_point(poly_it->inset_idx, poly_it, front);
                Point p = end_point.p();

                std::vector<PolylineEndRef> nearby_end_points = grid.getNearbyVals(p, snap_dist);
                for (PolylineEndRef& other_end : nearby_end_points)
                {
                    if (end_point == other_end
                        || other_end.polyline->junctions.empty()
                        || end_point.polyline->junctions.empty()
                        || !shorterThen(p - other_end.p(), snap_dist)
                    )
                    {
                        continue;
                    }
                    if (&*end_point.polyline == &*other_end.polyline)
                    { // we can close this polyline into a polygon
                        if (other_end.polyline->junctions.size() <= 2)
                        { // don't print single line segments double
                            continue;
                        }
                        result_polygons_per_index.resize(std::max(result_polygons_per_index.size(), static_cast<size_t>(end_point.inset_idx + 1)));
                        result_polygons_per_index[end_point.inset_idx].emplace_back(poly_it->junctions.begin(), poly_it->junctions.end());
                        end_point.polyline->junctions.clear();
                    }
                    else if (!end_point.front && other_end.front)
                    {
                        end_point.polyline->junctions.splice(end_point.polyline->junctions.end(), other_end.polyline->junctions);
                    }
                    else if (end_point.front && !other_end.front)
                    {
                        end_point.polyline->junctions.splice(end_point.polyline->junctions.begin(), other_end.polyline->junctions);
                    }
                    else if (end_point.front && other_end.front)
                    {
                        end_point.polyline->junctions.insert(end_point.polyline->junctions.begin(), other_end.polyline->junctions.rbegin(), other_end.polyline->junctions.rend());
                        other_end.polyline->junctions.clear();
                    }
                    else // if (!end_point.front && !other_end.front)
                    {
                        end_point.polyline->junctions.insert(end_point.polyline->junctions.end(), other_end.polyline->junctions.rbegin(), other_end.polyline->junctions.rend());
                        other_end.polyline->junctions.clear();
                    }
                }
            }
        }
    }

    // remove empties lists
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

void BeadingOrderOptimizer::reduceIntersectionOverlap(std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index)
{
    for (Polyline& polyline : odd_polylines)
    {
        reduceIntersectionOverlap(polyline, polyline.junctions.begin(), polyline.inset_idx, polygons_per_index);
        reduceIntersectionOverlap(polyline, polyline.junctions.rbegin(), polyline.inset_idx, polygons_per_index);
    }
}


template<typename directional_iterator>
void BeadingOrderOptimizer::reduceIntersectionOverlap(Polyline& polyline, directional_iterator polyline_start_it, coord_t inset_idx, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index)
{
    ExtrusionJunction& start_junction = *polyline_start_it;
    ExtrusionJunction* intersecting_junction = nullptr;
    if (inset_idx < polygons_per_index.size())
    {
        for (std::vector<ExtrusionJunction>& polygon : polygons_per_index[inset_idx])
        {
            for (ExtrusionJunction& junction : polygon)
            {
                if (junction.p == start_junction.p)
                {
                    assert(junction.w == start_junction.w);
                    intersecting_junction = &junction;
                    break;
                }
            }
            if (intersecting_junction) break;
        }
    }
    if (!intersecting_junction)
    {
        for (Polyline& other_polyline : odd_polylines)
        {
            if (other_polyline.inset_idx != polyline.inset_idx) continue;

            for (auto junction_it = ++other_polyline.junctions.begin(); junction_it != --other_polyline.junctions.end() && junction_it != other_polyline.junctions.end(); junction_it++)
            { // skip end points, because they would have been connected to this point!
                if (junction_it->p == start_junction.p)
                {
                    intersecting_junction = &*junction_it;
                    break;
                }
            }
            if (intersecting_junction) break;
        }
    }
    if (!intersecting_junction) return;

    reduceIntersectionOverlap(polyline, polyline_start_it, 0, intersecting_junction->w / 2 * (1.0 - intersection_overlap));
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

    if (traveled_dist + length > total_reduction_length)
    {
        Point mid1 = a + ab * std::max(static_cast<coord_t>(0), std::min(length, (total_reduction_length - traveled_dist) / length));
//         Point mid2 = mid1; // a + ab * std::min(reduction_length + 10, length) / length;
        std::list<ExtrusionJunction>::iterator forward_it = getInsertPosIt(next_junction_it);
        coord_t mid_w = start_junction.w + (next_junction.w - start_junction.w) * reduction_length / length;
        polyline.junctions.insert(forward_it, ExtrusionJunction(mid1, mid_w, start_junction.perimeter_index));
//         polyline.junctions.insert(forward_it, ExtrusionJunction(mid1, 0, start_junction.perimeter_index));
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
        polyline.junctions.emplace_back(polyline.junctions.front());
        polyline.junctions.back().p += Point(1,1);
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
                assert(junction.p.X < 100000 && junction.p.Y < 100000);
                assert(junction.p.X > -100000 && junction.p.Y > -100000);
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
