//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>
#include <queue> //Priority queue to prioritise removing unimportant vertices.

#include "Simplify.h"

namespace cura
{

Simplify::Simplify(const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation)
    : max_resolution(max_resolution)
    , max_deviation(max_deviation)
    , max_area_deviation(max_area_deviation)
{}

Simplify::Simplify(const Settings& settings)
    : max_resolution(settings.get<coord_t>("meshfix_maximum_resolution"))
    , max_deviation(settings.get<coord_t>("meshfix_maximum_deviation"))
    , max_area_deviation(settings.get<coord_t>("meshfix_maximum_area_deviation"))
{}

Polygon Simplify::polygon(const Polygon& polygon)
{
    constexpr bool is_closed = true;
    return simplify(polygon, is_closed);
}

ExtrusionLine Simplify::polygon(const ExtrusionLine& polygon)
{
    constexpr bool is_closed = true;
    return simplify(polygon, is_closed);
}

Polygon Simplify::polyline(const Polygon& polyline)
{
    constexpr bool is_closed = false;
    return simplify(polyline, is_closed);
}

size_t Simplify::nextNotDeleted(size_t index, const std::vector<bool>& to_delete) const
{
    const size_t size = to_delete.size();
    for(index = (index + 1) % size; to_delete[index]; index = (index + 1) % size); //Changes the index variable in-place until we found one that is not deleted.
    return index;
}

size_t Simplify::previousNotDeleted(size_t index, const std::vector<bool>& to_delete) const
{
    const size_t size = to_delete.size();
    for(index = (index + size - 1) % size; to_delete[index]; index = (index + size - 1) % size); //Changes the index variable in-place until we found one that is not deleted.
    return index;
}

void Simplify::appendVertex(Polygon& polygon, const Point& vertex) const
{
    polygon.add(vertex);
}

void Simplify::appendVertex(ExtrusionLine& extrusion_line, const ExtrusionJunction& vertex) const
{
    extrusion_line.junctions.push_back(vertex);
}

Point Simplify::getPosition(const Point& vertex) const
{
    return vertex;
}

Point Simplify::getPosition(const ExtrusionJunction& vertex) const
{
    return vertex.p;
}

Point Simplify::createIntersection(const Point& before, const Point intersection, const Point& after) const
{
    return intersection;
}

ExtrusionJunction Simplify::createIntersection(const ExtrusionJunction& before, const Point intersection, const ExtrusionJunction& after) const
{
    //Average the extrusion width of the line.
    //More correct would be to see where along the line the intersection occurs with a projection or something.
    //But these details are so small, and this solution is so much quicker and simpler.
    return ExtrusionJunction(intersection, (before.w + after.w) / 2, before.perimeter_index);
}

}