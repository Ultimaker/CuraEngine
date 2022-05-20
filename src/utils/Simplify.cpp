//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>
#include <queue> //Priority queue to prioritise removing unimportant vertices.

#include "Simplify.h"
#include "linearAlg2D.h"

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

coord_t Simplify::importance(const PolygonRef& polygon, const std::vector<bool>& to_delete, const size_t index, const bool is_closed) const
{
    const size_t poly_size = polygon.size();
    if(!is_closed && (index == 0 || index == poly_size - 1))
    {
        return std::numeric_limits<coord_t>::max(); //Endpoints of the polyline must always be retained.
    }
    //From here on out we can safely look at the vertex neighbours and assume it's a polygon. We won't go out of bounds of the polyline.

    const Point& vertex = polygon[index];
    const Point& before = polygon[previousNotDeleted(index, to_delete)];
    const Point& after = polygon[nextNotDeleted(index, to_delete)];
    const coord_t deviation2 = LinearAlg2D::getDist2FromLine(vertex, before, after);
    if(deviation2 <= min_resolution * min_resolution) //Deviation so small that it's always desired to remove them.
    {
        return deviation2;
    }
    if(vSize2(before - vertex) > max_resolution * max_resolution && vSize2(after - vertex) > max_resolution * max_resolution)
    {
        return std::numeric_limits<coord_t>::max(); //Long line segments, no need to remove this one.
    }
    return deviation2;
}

Polygon Simplify::polygon(const PolygonRef polygon)
{
    std::vector<bool> to_delete(polygon.size(), false);
    auto comparator = [this, polygon, to_delete](const size_t vertex_a, const size_t vertex_b)
    {
        return compare(polygon, to_delete, vertex_a, vertex_b, true);
    };
    std::priority_queue<size_t, std::vector<size_t>, decltype(comparator)> kept_vertices(comparator);

    //Add the initial points.
    for(size_t i = 0; i < polygon.size(); ++i)
    {
        kept_vertices.emplace(i);
    }

    //TODO: Remove vertices to simplify the polygon.

    return polygon; //TODO.
}

bool Simplify::compare(const PolygonRef& polygon, const std::vector<bool>& to_delete, const size_t vertex_a, const size_t vertex_b, const bool is_closed) const
{
    const coord_t importance_a = importance(polygon, to_delete, vertex_a, is_closed);
    const coord_t importance_b = importance(polygon, to_delete, vertex_b, is_closed);
    return importance_a < importance_b;
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

}