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
        return path->outline.outerPolygon();
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
        for (const ExtrusionJunction junction : *path)
        {
            poly.add(junction.p);
        }
        return ConstPolygonRef(poly);
    }

}
