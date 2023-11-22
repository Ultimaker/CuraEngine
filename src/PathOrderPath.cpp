// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrdering.h" //The definitions we're implementing here.
#include "WallToolPaths.h"
#include "sliceDataStorage.h" //For SliceLayerPart.

namespace cura
{

template<>
ConstPolygonRef PathOrdering<ConstPolygonPointer>::getVertexData()
{
    return *vertices_;
}

template<>
ConstPolygonRef PathOrdering<PolygonPointer>::getVertexData()
{
    return *vertices_;
}

template<>
ConstPolygonRef PathOrdering<const SkinPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrdering<const SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrdering<const SupportInfillPart*>::getVertexData()
{
    return vertices_->outline_.outerPolygon();
}
template<>
ConstPolygonRef PathOrdering<const ExtrusionLine*>::getVertexData()
{
    if (! cached_vertices_)
    {
        cached_vertices_ = vertices_->toPolygon();
    }
    return ConstPolygonRef(*cached_vertices_);
}

} // namespace cura
