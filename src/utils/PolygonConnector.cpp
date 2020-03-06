/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */

#include "PolygonConnector.h"

#include "linearAlg2D.h"
#include "AABB.h"

namespace cura 
{

Polygons PolygonConnector::connect()
{
    Polygons ret;

    if (input_polygons.empty())
    {
        return ret;
    }

    std::vector<Polygon> to_connect;
    to_connect.reserve(input_polygons.size());
    for (ConstPolygonPointer poly : input_polygons)
    {
        to_connect.emplace_back(*poly); // copy into list
    }

    while (!to_connect.empty())
    {
        if (to_connect.size() == 1)
        {
            ret.add(std::move(to_connect.back()));
            break;
        }
        Polygon current = std::move(to_connect.back());
        to_connect.pop_back();

        std::optional<PolygonBridge> bridge = getBridge(current, to_connect);
        if (bridge)
        {
            all_bridges.push_back(*bridge); // just for keeping scores
            // remove other poly from the list and put the newly connected one on the list
            // i.e. replace the old other poly by the new one
            PolygonRef other_poly(*const_cast<ClipperLib::Path*>(bridge->a.to.poly.operator->())); // const casting a ConstPolygonPointer is difficult!
            other_poly = std::move(connectPolygonsAlongBridge(*bridge)); // connect the bridged parts and overwrite the other polygon with it.

            // don't store the current poly, it has just been connected and stored
        }
        else
        {
            ret.add(std::move(current));
        }
    }

    return ret;
}


Polygon PolygonConnector::connectPolygonsAlongBridge(const PolygonConnector::PolygonBridge& bridge)
{
    // enforce the following orientations:
    //
    // <<<<<<X......X<<<<<<< poly2
    //       ^      v
    //       ^      v
    //       ^ a  b v bridge
    //       ^      v
    // >>>>>>X......X>>>>>>> poly1
    //
    // this should work independent from whether it is a hole polygon or a outline polygon
    Polygon ret;
    addPolygonSegment(bridge.b.from, bridge.a.from, ret);
    addPolygonSegment(bridge.a.to, bridge.b.to, ret);
    return ret;
}

void PolygonConnector::addPolygonSegment(const ClosestPolygonPoint& start, const ClosestPolygonPoint& end, PolygonRef result)
{
    // <<<<<<<.start     end.<<<<<<<<
    //        ^             v
    //        ^             v
    // >>>>>>>.end.....start.>>>>>>>
    assert(start.poly == end.poly && "We can only bridge from one polygon from the other if both connections depart from the one polygon!");
    ConstPolygonRef poly = *end.poly;
    int16_t dir = getPolygonDirection(end, start); // we get the direction of the polygon in between the bridge connections, while we add the segment of the polygon not in between the segments

    result.add(start.p());
    bool first_iter = true;
    for (size_t vert_nr = 0; vert_nr < poly.size(); vert_nr++)
    {
        size_t vert_idx =
            (dir > 0)?
            (start.point_idx + 1 + vert_nr) % poly.size()
            : (static_cast<size_t>(start.point_idx) - vert_nr + poly.size()) % poly.size(); // cast in order to accomodate subtracting
        if (!first_iter // don't return without adding points when the starting point and ending point are on the same polygon segment
            && vert_idx == (end.point_idx + ((dir > 0)? 1 : 0)) % poly.size())
        { // we've added all verts of the original polygon segment between start and end
            break;
        }
        first_iter = false;
        result.add(poly[vert_idx]);
    }
    result.add(end.p());
}

int16_t PolygonConnector::getPolygonDirection(const ClosestPolygonPoint& from, const ClosestPolygonPoint& to)
{
    assert(from.poly == to.poly && "We can only bridge from one polygon from the other if both connections depart from the one polygon!");
    ConstPolygonRef poly = *from.poly;
    if (from.point_idx == to.point_idx)
    {
        const Point prev_vert = poly[from.point_idx];
        const coord_t from_dist2 = vSize2(from.p() - prev_vert);
        const coord_t to_dist2 = vSize2(to.p() - prev_vert);
        return (to_dist2 > from_dist2)? 1 : -1;
    }
    // TODO: replace naive implementation by robust implementation
    // naive idea: there are less vertices in between the connection points than around
    const size_t a_to_b_vertex_count = (static_cast<size_t>(to.point_idx) - from.point_idx + poly.size()) % poly.size(); // cast in order to accomodate subtracting
    if (a_to_b_vertex_count > poly.size() / 2)
    {
        return -1; // from a to b is in the reverse direction as the vertices are saved in the polygon
    }
    else
    {
        return 1; // from a to b is in the same direction as the vertices are saved in the polygon
    }
}

std::optional<PolygonConnector::PolygonBridge> PolygonConnector::getBridge(ConstPolygonRef from_poly, std::vector<Polygon>& to_polygons)
{
    // line distance between consecutive polygons should be at least the line_width
    const coord_t min_connection_length = line_width - 10;
    // Only connect polygons if they are next to each other
    // allow for a bit of wiggle room for when the polygons are very curved,
    // which causes any connection distance to be larger than the offset amount
    const coord_t max_connection_length = line_width * 3 / 2;

    std::optional<PolygonConnector::PolygonConnection> first_connection;
    std::optional<PolygonConnector::PolygonConnection> second_connection;

    Polygons to_polys;
    for (const Polygon& poly : to_polygons)
    {
        to_polys.add(poly);
    }

    std::function<bool (std::pair<ClosestPolygonPoint, ClosestPolygonPoint>)> can_make_bridge =
        [&, this](std::pair<ClosestPolygonPoint, ClosestPolygonPoint> candidate)
        {
            first_connection.emplace(candidate.first, candidate.second);
            first_connection->to.poly = &to_polygons[first_connection->to.poly_idx]; // because it still refered to the local variable [to_polys]
            if (first_connection->getDistance2() > max_dist * max_dist)
            {
                return false;
            }
            second_connection = PolygonConnector::getSecondConnection(*first_connection);
            if (!second_connection)
            {
                return false;
            }
            return true;
        };

    std::pair<ClosestPolygonPoint, ClosestPolygonPoint> connection_points = PolygonUtils::findConnection(from_poly, to_polys, min_connection_length, max_connection_length, can_make_bridge);

    if (!connection_points.first.isValid() || !connection_points.second.isValid())
    { // We didn't find a connection which can make a bridge
        // We might have found a first_connection and a second_connection, but maybe they didn't satisfy all criteria.
        std::optional<PolygonConnector::PolygonBridge> uninitialized;
        return uninitialized;
    }

    if (first_connection && second_connection)
    {
        PolygonBridge result(*first_connection, *second_connection);
        // ensure that b is always the right connection and a the left
        Point a_vec = result.a.to.p() - result.a.from.p();
        Point shift = turn90CCW(a_vec);
        if (dot(shift, result.b.from.p() - result.a.from.p()) > 0)
        {
            std::swap(result.a, result.b);
        }
        return result;
    }
    else
    { // we couldn't find a connection; maybe the polygon was too small or maybe it is just too far away from other polygons.
        return std::optional<PolygonConnector::PolygonBridge>();
    }
}

std::optional<PolygonConnector::PolygonConnection> PolygonConnector::getSecondConnection(PolygonConnection& first)
{
    bool forward = true;
    std::optional<ClosestPolygonPoint> from_a = PolygonUtils::getNextParallelIntersection(first.from, first.to.p(), line_width, forward);
    if (!from_a)
    { // then there's also not going to be a b
        return std::optional<PolygonConnector::PolygonConnection>();
    }
    std::optional<ClosestPolygonPoint> from_b = PolygonUtils::getNextParallelIntersection(first.from, first.to.p(), line_width, !forward);

    std::optional<ClosestPolygonPoint> to_a = PolygonUtils::getNextParallelIntersection(first.to, first.from.p(), line_width, forward);
    if (!to_a)
    {
        return std::optional<PolygonConnector::PolygonConnection>();
    }
    std::optional<ClosestPolygonPoint> to_b = PolygonUtils::getNextParallelIntersection(first.to, first.from.p(), line_width, !forward);


    const Point shift = turn90CCW(first.from.p() - first.to.p());
    
    std::optional<PolygonConnection> best;
    coord_t best_total_distance2 = std::numeric_limits<coord_t>::max();
    for (unsigned int from_idx = 0; from_idx < 2; from_idx++)
    {
        std::optional<ClosestPolygonPoint> from_opt = (from_idx == 0)? from_a : from_b;
        if (!from_opt)
        {
            continue;
        }
        for (unsigned int to_idx = 0; to_idx < 2; to_idx++)
        {
            std::optional<ClosestPolygonPoint> to_opt = (to_idx == 0)? to_a : to_b;
            // for each combination of from and to
            if (!to_opt)
            {
                continue;
            }
            ClosestPolygonPoint from = *from_opt;
            ClosestPolygonPoint to = *to_opt;

            coord_t from_projection = dot(from.p() - first.to.p(), shift);
            coord_t to_projection = dot(to.p() - first.to.p(), shift);
            if (from_projection * to_projection <= 0)
            { // ends lie on different sides of the first connection
                continue;
            }

            const coord_t total_distance2 = vSize2(from.p() - to.p()) + vSize2(from.p() - first.from.p()) + vSize(to.p() - first.to.p());
            if (total_distance2 < best_total_distance2)
            {
                best.emplace(from, to);
                best_total_distance2 = total_distance2;
            }
            
            if (to_opt == to_b)
            {
                break;
            }
        }
        if (from_opt == from_b)
        {
            break;
        }
    }
    if (!best || best_total_distance2 > max_dist * max_dist + 2 * (line_width + 10) * (line_width + 10))
    {
        return std::optional<PolygonConnector::PolygonConnection>();
    }
    else
    {
        return *best;
    }
}


}//namespace cura

