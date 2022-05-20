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

coord_t Simplify::importance(const PolygonRef& polygon, const Point& point, const size_t vertex, const bool is_closed) const
{
    const size_t poly_size = polygon.size();
    if(!is_closed && (vertex == 0 || vertex == poly_size - 1))
    {
        return std::numeric_limits<coord_t>::max(); //Endpoints of the polyline must always be retained.
    }
    //From here on out we can safely look at the vertex neighbours and assume it's a polygon. We won't go out of bounds of the polyline.

    const Point& before = polygon[(vertex + poly_size - 1) % poly_size];
    const Point& after = polygon[(vertex + 1) % poly_size];
    const coord_t deviation2 = LinearAlg2D::getDist2FromLine(point, before, after);
    if(deviation2 <= min_resolution * min_resolution) //Deviation so small that it's always desired to remove them.
    {
        return deviation2;
    }
    if(vSize2(before - point) > max_resolution * max_resolution && vSize2(after - point) > max_resolution * max_resolution)
    {
        return std::numeric_limits<coord_t>::max(); //Long line segments, no need to remove this one.
    }
    return deviation2;
}

Polygon Simplify::polygon(const PolygonRef polygon)
{
    auto comparator = [this](const PolygonVertex& vertex_a, const PolygonVertex& vertex_b)
    {
        return compare(vertex_a, vertex_b);
    };
    std::priority_queue<PolygonVertex, std::vector<PolygonVertex>, decltype(comparator)> kept_vertices(comparator);

    //Add the initial points.
    for(size_t i = 0; i < polygon.size(); ++i)
    {
        kept_vertices.emplace(i, polygon[i], &polygon);
    }

    //TODO: Remove vertices to simplify the polygon.

    return polygon; //TODO.
}

bool Simplify::compare(const PolygonVertex& vertex_a, const PolygonVertex& vertex_b) const
{
    const coord_t importance_a = importance(*vertex_a.polygon, vertex_a.position, vertex_a.index, true);
    const coord_t importance_b = importance(*vertex_b.polygon, vertex_b.position, vertex_b.index, true);
    return importance_a < importance_b;
}

}