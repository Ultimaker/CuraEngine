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
    std::vector<std::vector<MonotoneVertexType>> vertex_categories = categorize(polygons);

    //Put all vertices in a priority queue, sorted from left to right for the scanning.
    auto compare_by_index = [&polygons](const VertexRef a, const VertexRef b)
    {
        const Point a_pos = polygons[a.first][a.second];
        const Point b_pos = polygons[b.first][b.second];
        return a_pos.X < b_pos.X || (a_pos.X == b_pos.X && a_pos.Y < b_pos.Y);
    };
    std::priority_queue<VertexRef, std::vector<VertexRef>, decltype(compare_by_index)> queue(compare_by_index);
    for(size_t poly_index = 0; poly_index < polygons.size(); ++poly_index)
    {
        for(size_t vertex_index = 0; vertex_index < polygons[poly_index].size(); ++vertex_index)
        {
            queue.push(std::pair<size_t, size_t>(poly_index, vertex_index));
        }
    }

    //Create a scanline data structure.
    //To achieve an O(n log(n)) algorithm this would have to be a binary search tree.
    //But practically the scan line is so small that we'll just use a vector and search linearly.
    std::vector<EdgeRef> scanline;

    //Store connections that we found here. We'll use this later to reconstruct the monotone polygons.
    std::unordered_map<VertexRef, size_t, VertexRefHash> connected_to;

    //Handle all vertices in order from left to right, adding connections as we go.
    while(!queue.empty())
    {
        const VertexRef vertex = queue.top();
        queue.pop();
        const size_t poly_index = vertex.first;
        const size_t poly_size = polygons[poly_index].size();
        const MonotoneVertexType category = vertex_categories[poly_index][vertex.second];
        switch(category)
        {
            case MonotoneVertexType::START:
                //Left of both of its neighbours. Add the edge that is higher to the scanline.
                scanline.push_back(EdgeRef{{poly_index, vertex.second}, {poly_index, (vertex.second + poly_size - 1) % poly_size}});
                break;
            case MonotoneVertexType::END:
                //Find the edge in the scanline that ends here and remove it.
                break;
        }
    }

    //TODO: Implement
    return std::vector<Polygon>();
}

void Triangulate::addTriangles(const Polygon& monotone_polygon, std::vector<Point>& result)
{
    //TODO: Implement
}

}
