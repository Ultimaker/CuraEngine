//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Triangulate.h" //The class we're implementing.

#include <earcut/earcut.hpp> //To do the triangulation itself.

namespace mapbox
{
namespace util
{

//Translation for Earcut from Clipper's data structures.
template<>
struct nth<0, cura::Point> {
    inline static cura::coord_t get(const cura::Point& point) {
        return point.X;
    }
};

template<>
struct nth<1, cura::Point> {
    inline static cura::coord_t get(const cura::Point& point) {
        return point.Y;
    }
};

}
}

namespace cura
{

std::vector<Point> Triangulate::triangulate(const Polygons& polygons)
{
    std::vector<Point> result;
    result.reserve(polygons.pointCount() * 3); //The result should be slightly smaller than this, so it's a good estimate.

    //Format this Polygons into a format that Earcut understands.
    std::vector<PolygonsPart> parts = polygons.splitIntoParts(true);
    for(const PolygonsPart& part : parts)
    {
        std::vector<size_t> indices = mapbox::earcut<size_t>(part);
        for(uint32_t index : indices) //Find the actual vertex position back.
        {
            size_t poly_index = 0;
            while(index >= part[poly_index].size())
            {
                index -= part[poly_index].size();
                ++poly_index;
            }
            result.push_back(part[poly_index][index]);
        }
    }

    return result;
}

}
