//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Triangulate.h" //The class we're implementing.

#include "linearAlg2D.h"
#include "polygon.h"

namespace cura
{

std::vector<Point> Triangulate::triangulate(const Polygons& polygons)
{
    std::vector<Point> result;
    result.reserve(polygons.pointCount()); //The result should be slightly smaller than this, so it's a good estimate.

    std::vector<Polygon> monotone_parts = splitXMonotone(polygons);
    for(const Polygon& part : monotone_parts)
    {
        addTriangles(part, result);
    }

    return result;
}

std::vector<std::vector<Triangulate::MonotoneVertexType>> Triangulate::categorize(const Polygons& polygons)
{
    std::vector<std::vector<Triangulate::MonotoneVertexType>> result;
    result.reserve(polygons.size());
    for(const std::vector<ClipperLib::IntPoint>& polygon : polygons)
    {
        result.emplace_back();
        std::vector<Triangulate::MonotoneVertexType>& categorization = result.back();
        categorization.reserve(polygon.size());
        for(size_t i = 0; i < polygon.size(); ++i)
        {
            const size_t next = (i + 1) % polygon.size();
            const size_t previous = (i + polygon.size() - 1) % polygon.size();
            if(polygon[i].X < polygon[next].X || (polygon[i].X == polygon[next].X && polygon[i].Y < polygon[next].Y))
            {
                if(polygon[i].X < polygon[previous].X || (polygon[i].X == polygon[previous].X && polygon[i].Y < polygon[previous].Y))
                {
                    //i is left of both its neighbors.
                    if(LinearAlg2D::pointIsLeftOfLine(polygon[next], polygon[previous], polygon[i]) < 0)
                    {
                        categorization.push_back(MonotoneVertexType::START);
                    }
                    else
                    {
                        categorization.push_back(MonotoneVertexType::SPLIT);
                    }
                }
                else
                {
                    categorization.push_back(MonotoneVertexType::REGULAR);
                }
            }
            else
            {
                if(polygon[i].X > polygon[previous].X || (polygon[i].X == polygon[previous].X && polygon[i].Y > polygon[previous].Y))
                {
                    //i is right of both its neighbours.
                    if(LinearAlg2D::pointIsLeftOfLine(polygon[next], polygon[previous], polygon[i]) < 0)
                    {
                        categorization.push_back(MonotoneVertexType::END);
                    }
                    else
                    {
                        categorization.push_back(MonotoneVertexType::MERGE);
                    }
                }
                else
                {
                    categorization.push_back(MonotoneVertexType::REGULAR);
                }
            }
        }
    }
    return result;
}

std::vector<Polygon> Triangulate::splitXMonotone(const Polygons& polygons)
{
    //TODO: Implement
    return std::vector<Polygon>();
}

void Triangulate::addTriangles(const Polygon& monotone_polygon, std::vector<Point>& result)
{
    //TODO: Implement
}

}
