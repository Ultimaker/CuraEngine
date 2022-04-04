//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Triangulate.h"
#include "polygon.h"

namespace cura
{

std::vector<Point> Triangulate::triangulate(const Polygons& polygons)
{
    std::vector<Point> result;
    result.reserve(polygons.pointCount()); //The result should be slightly smaller than this, so it's a good estimate.

    std::vector<Polygon> monotone_parts = splitYMonotone(polygons);
    for(const Polygon& part : monotone_parts)
    {
        addTriangles(part, result);
    }

    return result;
}

std::vector<Polygon> Triangulate::splitYMonotone(const Polygons& polygons)
{
    //TODO: Implement
    return std::vector<Polygon>();
}

void Triangulate::addTriangles(const Polygon& monotone_polygon, std::vector<Point>& result)
{
    //TODO: Implement
}

}
