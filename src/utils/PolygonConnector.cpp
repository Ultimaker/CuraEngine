//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonConnector.h"

#include "linearAlg2D.h"
#include "AABB.h"

namespace cura 
{

PolygonConnector::PolygonConnector(const coord_t line_width, const coord_t max_dist)
: line_width(line_width - 5) // a bit less so that consecutive lines which have become connected can still connect to other lines
//                |                     |                      |
// ----------o    |      ----------o    |       ----------o,,,,o
//           |    |  ==>           |    |  ==>
// -----o    |    |      -----o----o    |       -----o----o----o
//      |    |    |                     |                      |
//      |    |    |           o''''o    |            o''''o    |
//      |    |    |           |    |    |            |    |    |
, max_dist(max_dist)
{}

void PolygonConnector::add(const Polygons& input)
{
    for (ConstPolygonRef poly : input)
    {
        input_polygons.push_back(poly);
    }
}

void PolygonConnector::add(const VariableWidthPaths& input)
{
    for(const VariableWidthLines& lines : input)
    {
        for(const ExtrusionLine& line : lines)
        {
            input_paths.push_back(line);
        }
    }
}

void PolygonConnector::connect(Polygons& output_polygons, VariableWidthPaths& output_paths)
{
    std::vector<Polygon> result_polygons = connectGroup(input_polygons);
    for(Polygon& polygon : result_polygons)
    {
        output_polygons.add(polygon);
    }
}

Point PolygonConnector::getPosition(const Point& vertex) const
{
    return vertex;
}

Point PolygonConnector::getPosition(const ExtrusionJunction& junction) const
{
    return junction.p;
}

coord_t PolygonConnector::getWidth(const Point&) const
{
    return line_width;
}

coord_t PolygonConnector::getWidth(const ExtrusionJunction& junction) const
{
    return junction.w;
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

}//namespace cura

