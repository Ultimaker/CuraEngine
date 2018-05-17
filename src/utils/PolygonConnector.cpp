/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */

#include "PolygonConnector.h"

#include "linearAlg2D.h"

namespace cura 
{

Polygons PolygonConnector::connect()
{
    logError("Unimplemented!\n");
}


Polygon PolygonConnector::connect(const PolygonConnector::PolygonBridge& bridge)
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
    //        ^             v
    //        ^             v
    // >>>>>>>.end.....start.>>>>>>>
    assert(a.poly == b.poly && "We can only bridge from one polygon from the other if both connections depart from the one polygon!");
    ConstPolygonRef poly = *end.poly;
    char dir = getPolygonDirection(end, start); // we get the direction of the polygon in between the bridge connections, while we add the segment of the polygon not in between the segments

    result.add(start.p());
    bool first_iter = true;
    for (size_t vert_nr = 0; vert_nr < poly.size(); vert_nr++)
    {
        size_t vert_idx =
            (dir > 0)?
            (start.point_idx + 1 + vert_nr) % poly.size()
            : (start.point_idx - vert_nr + poly.size()) % poly.size();
        if (!first_iter && vert_idx == (end.point_idx + ((dir > 0)? 1 : 0)) % poly.size())
        {
            break;
        }
        first_iter = false;
        result.add(poly[vert_idx]);
    }
    result.add(end.p());
}

char PolygonConnector::getPolygonDirection(const ClosestPolygonPoint& from, const ClosestPolygonPoint& to)
{
    assert(a.poly == b.poly && "We can only bridge from one polygon from the other if both connections depart from the one polygon!");
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
    const size_t a_to_b_vertex_count = (to.point_idx - from.point_idx + poly.size()) % poly.size();
    if (a_to_b_vertex_count > poly.size() / 2)
    {
        return 1; // from a to b is in the reverse direction as the vertices are saved in the polygon
    }
    else
    {
        return -1; // from a to b is in the same direction as the vertices are saved in the polygon
    }
}

std::optional<PolygonConnector::PolygonBridge> PolygonConnector::getBridge(ConstPolygonRef from_poly, std::vector<Polygon>& to_polygons)
{
    std::optional<PolygonConnector::PolygonConnection> connection = getConnection(from_poly, to_polygons);
    if (!connection || connection->getDistance2() > max_dist * max_dist)
    {
        return std::optional<PolygonConnector::PolygonBridge>();
    }

    // try to get the other connection forward
    std::optional<PolygonConnector::PolygonConnection> second_connection = PolygonConnector::getSecondConnection(*connection);
    if (connection && second_connection)
    {
        PolygonBridge result;
        result.a = *connection;
        result.b = *second_connection;
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
    { // the polygon is too small to have a bridge attached from [connection]
        // TODO: try fitting the bridge from the extrema of the poly
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
    
    PolygonConnection best;
    coord_t best_distance2 = std::numeric_limits<coord_t>::max();
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

            const coord_t distance2 = vSize2(from.p() - to.p());
            if (distance2 < best_distance2)
            {
                best.from = from;
                best.to = to;
                best_distance2 = distance2;
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
    if (best_distance2 == std::numeric_limits<coord_t>::max())
    {
        return std::optional<PolygonConnector::PolygonConnection>();
    }
    else
    {
        return best;
    }
}


std::optional<PolygonConnector::PolygonConnection> PolygonConnector::getConnection(ConstPolygonRef from_poly, std::vector<Polygon>& to_polygons)
{
    constexpr int sample_size = 6; //!< TODO: hardcoded sample size parameter!
    PolygonConnection best_connection;
    coord_t best_connection_distance2 = std::numeric_limits<coord_t>::max();
    ClosestPolygonPoint from_location(from_poly);
    for (ConstPolygonRef to_poly : to_polygons)
    {
        if (to_poly.data() == from_poly.data())
        { // don't connect a polygon to itself
            continue;
        }
        ClosestPolygonPoint to_location(to_poly);
        PolygonUtils::findSmallestConnection(from_location, to_location, sample_size);

        coord_t connection_distance2 = vSize2(to_location.p() - from_location.p());
        if (connection_distance2 < best_connection_distance2)
        {
            best_connection.from = from_location;
            best_connection.to = to_location;
            best_connection_distance2 = connection_distance2;
            if (connection_distance2 < (line_width + 10) * (line_width + 10))
            {
                return best_connection;
            }
        }
    }
    if (best_connection_distance2 == std::numeric_limits<coord_t>::max())
    {
        return std::optional<PolygonConnector::PolygonConnection>();
    }
    else
    {
        return best_connection;
    }
}


}//namespace cura

