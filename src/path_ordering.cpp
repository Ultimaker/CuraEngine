// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "path_ordering.h" //The definitions we're implementing here.

#include "WallToolPaths.h"
#include "sliceDataStorage.h" //For SliceLayerPart.

namespace cura
{

template<>
const Polygon& PathOrdering<const Polygon*>::getVertexData()
{
    return *vertices_;
}

template<>
const Polygon& PathOrdering<Polygon*>::getVertexData()
{
    return *vertices_;
}

template<>
const Polygon& PathOrdering<const OpenPolyline*>::getVertexData()
{
    return *reinterpret_cast<const Polygon*>(vertices_);
}

template<>
const Polygon& PathOrdering<OpenPolyline*>::getVertexData()
{
    return *reinterpret_cast<Polygon*>(vertices_);
}

template<>
const Polygon& PathOrdering<const SkinPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const Polygon& PathOrdering<const SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const Polygon& PathOrdering<const SupportInfillPart*>::getVertexData()
{
    return vertices_->outline_.outerPolygon();
}
template<>
const Polygon& PathOrdering<const ExtrusionLine*>::getVertexData()
{
    if (! cached_vertices_)
    {
        cached_vertices_ = vertices_->toPolygon();
    }
    return *cached_vertices_;
}

} // namespace cura
