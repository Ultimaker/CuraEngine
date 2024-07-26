// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonConnector.h"

#include "utils/AABB.h"
#include "utils/linearAlg2D.h"

namespace cura
{

PolygonConnector::PolygonConnector(const coord_t line_width)
    : line_width_(line_width)
{
}

void PolygonConnector::add(const Shape& input)
{
    for (const Polygon& poly : input)
    {
        input_polygons_.push_back(poly);
    }
}

void PolygonConnector::add(const std::vector<VariableWidthLines>& input)
{
    for (const VariableWidthLines& lines : input)
    {
        for (const ExtrusionLine& line : lines)
        {
            input_paths_.push_back(line);
        }
    }
}

void PolygonConnector::connect(Shape& output_polygons, std::vector<VariableWidthLines>& output_paths)
{
    std::vector<Polygon> result_polygons = connectGroup(input_polygons_);
    for (const Polygon& polygon : result_polygons)
    {
        output_polygons.push_back(polygon);
    }

    std::vector<ExtrusionLine> result_paths = connectGroup(input_paths_);
    output_paths.push_back(result_paths);
}

Point2LL PolygonConnector::getPosition(const Point2LL& vertex) const
{
    return vertex;
}

Point2LL PolygonConnector::getPosition(const ExtrusionJunction& junction) const
{
    return junction.p_;
}

coord_t PolygonConnector::getWidth(const Point2LL&) const
{
    return line_width_;
}

coord_t PolygonConnector::getWidth(const ExtrusionJunction& junction) const
{
    return junction.w_;
}

void PolygonConnector::addVertex(Polygon& polygonal, const Point2LL& position, const coord_t) const
{
    polygonal.push_back(position);
}

void PolygonConnector::addVertex(Polygon& polygonal, const Point2LL& vertex) const
{
    polygonal.push_back(vertex);
}

void PolygonConnector::addVertex(ExtrusionLine& polygonal, const Point2LL& position, const coord_t width) const
{
    polygonal.emplace_back(position, width, 1); // Perimeter indices don't make sense any more once perimeters are merged. Use 1 as placeholder, being the first "normal" wall.
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
    constexpr size_t inset_index = 1; // Specialising to set inset_index to 1 instead of maximum int. Connected polys are not specific to any inset.
    constexpr bool is_odd = false;
    ExtrusionLine result(inset_index, is_odd);
    result.is_closed_ = true;
    return result; // No copy, via RVO.
}

} // namespace cura
