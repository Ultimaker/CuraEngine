//Copyright (c) 2023 UltiMaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrdering.h" //The definitions we're implementing here.
#include "WallToolPaths.h"
#include "sliceDataStorage.h" //For SliceLayerPart.

namespace cura
{

    template<>
    ConstPolygonRef PathOrdering<ConstPolygonPointer>::getVertexData()
    {
        return *vertices;
    }

    template<>
    ConstPolygonRef PathOrdering<PolygonPointer>::getVertexData()
    {
        return *vertices;
    }

    template<>
    ConstPolygonRef PathOrdering<const SkinPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }

    template<>
    ConstPolygonRef PathOrdering<const SliceLayerPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }

    template<>
    ConstPolygonRef PathOrdering<const SupportInfillPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }
    template<>
    ConstPolygonRef PathOrdering<const ExtrusionLine*>::getVertexData()
    {
        if ( ! cached_vertices)
        {
            cached_vertices = vertices->toPolygon();
        }
        return ConstPolygonRef(*cached_vertices);
    }

}
