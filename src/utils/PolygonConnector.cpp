//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonConnector.h"

#include "utils/linearAlg2D.h"
#include "utils/AABB.h"

namespace cura 
{

PolygonConnector::PolygonConnector(const coord_t line_width)
: line_width(line_width)
{}

void PolygonConnector::add(const Polygons& input)
{
    for (ConstPolygonRef poly : input)
    {
        input_polygons.push_back(poly);
    }
}

void PolygonConnector::add(const std::vector<VariableWidthLines>& input)
{
    for(const VariableWidthLines& lines : input)
    {
        for(const ExtrusionLine& line : lines)
        {
            input_paths.push_back(line);
        }
    }
}

void PolygonConnector::connect(Polygons& output_polygons, std::vector<VariableWidthLines>& output_paths)
{
    std::vector<Polygon> result_polygons = connectGroup(input_polygons);
    for(Polygon& polygon : result_polygons)
    {
        output_polygons.add(polygon);
    }

    std::vector<ExtrusionLine> result_paths = connectGroup(input_paths);
    output_paths.push_back(result_paths);
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

void PolygonConnector::addVertex(Polygon& polygonal, const Point& position, const coord_t) const
{
    polygonal.add(position);
}

void PolygonConnector::addVertex(Polygon& polygonal, const Point& vertex) const
{
    polygonal.add(vertex);
}

void PolygonConnector::addVertex(ExtrusionLine& polygonal, const Point& position, const coord_t width) const
{
    polygonal.emplace_back(position, width, 1); //Perimeter indices don't make sense any more once perimeters are merged. Use 1 as placeholder, being the first "normal" wall.
}

void PolygonConnector::addVertex(ExtrusionLine& polygonal, const ExtrusionJunction& vertex) const
{
    polygonal.emplace_back(vertex);
}

bool PolygonConnector::isClosed(Polygon&) const
{
    return true;
}

bool PolygonConnector::isClosed(ExtrusionLine& polygonal) const
{
    return vSize2(polygonal.front() - polygonal.back()) < 25;
}

template<>
ExtrusionLine PolygonConnector::createEmpty<ExtrusionLine>() const
{
    constexpr size_t inset_index = 1; //Specialising to set inset_index to 1 instead of maximum int. Connected polys are not specific to any inset.
    constexpr bool is_odd = false;
    ExtrusionLine result(inset_index, is_odd);
    result.is_closed = true;
    return result; //No copy, via RVO.
}

}//namespace cura

