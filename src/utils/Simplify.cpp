//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>

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

coord_t Simplify::importance(const PolygonRef& polygon, const Point& point, const size_t vertex, const bool is_closed)
{
    const size_t poly_size = polygon.size();
    if(!is_closed && (vertex == 0 || vertex == poly_size - 1))
    {
        return std::numeric_limits<coord_t>::max(); //Endpoints of the polyline must always be retained.
    }
    //From here on out we can safely look at the vertex neighbours and assume it's a polygon. We won't go out of bounds of the polyline.

    const Point& before = polygon[(vertex + poly_size - 1) % poly_size];
    const Point& after = polygon[(vertex + 1) % poly_size];
    if(vSize2(before - point) > max_resolution * max_resolution && vSize2(after - point) > max_resolution * max_resolution)
    {
        return std::numeric_limits<coord_t>::max(); //Long line segments, no need to remove this one.
    }
    return LinearAlg2D::getDist2FromLine(point, before, after); //Return simple deviation.
}

}