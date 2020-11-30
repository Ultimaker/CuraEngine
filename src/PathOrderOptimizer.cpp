//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrderOptimizer.h" //The definitions we're implementing here.
#include "sliceDataStorage.h" //For SliceLayerPart.
#include "WallToolPaths.h"

//Since the PathOrderOptimizer is a template class, we will only implement the template specializations in this file.

namespace cura
{

template<>
ConstPolygonRef PathOrderOptimizer<ConstPolygonRef>::getVertexData(ConstPolygonRef path)
{
    return path;
}

template<>
ConstPolygonRef PathOrderOptimizer<PolygonRef>::getVertexData(PolygonRef path)
{
    return path;
}

template<>
ConstPolygonRef PathOrderOptimizer<const SkinPart*>::getVertexData(const SkinPart* path)
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrderOptimizer<const SliceLayerPart*>::getVertexData(const SliceLayerPart* path)
{
    if(!path->wall_toolpaths.empty())
    {
        Polygons poly;
        // Assuming the first wall tool path is always the outer wall
        VariableWidthPaths outer_wall { path->wall_toolpaths.front() };

        // Half the minimum wall line width, should be the minimum required stitch distance
        const coord_t stitch_distance = std::min_element(outer_wall.back().cbegin(), outer_wall.back().cend(),
                                       [](const ExtrusionLine& l, const ExtrusionLine& r)
                                       {
                                           return l.getWidth() < r.getWidth();
                                       })->getWidth() / 2;

        // Stitch the outer_wall contour to a polygon and store it in the cached vertices
        WallToolPaths::stitchContours(outer_wall, stitch_distance, poly);
        cached_vertices.emplace_back(poly.back());
        return ConstPolygonRef(cached_vertices.back());
    }
    else
    {
        return path->outline.outerPolygon();
    }
}

template<>
ConstPolygonRef PathOrderOptimizer<const SupportInfillPart*>::getVertexData(const SupportInfillPart* path)
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrderOptimizer<const LineJunctions*>::getVertexData(const LineJunctions* path)
{
    cached_vertices.emplace_back();
    Polygon& poly = cached_vertices.back();
    for(const ExtrusionJunction junction : *path)
    {
        poly.add(junction.p);
    }
    return ConstPolygonRef(poly);
}

}
