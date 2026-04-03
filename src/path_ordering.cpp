// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "path_ordering.h" //The definitions we're implementing here.

#include "WallToolPaths.h"
#include "geometry/OpenPolyline.h"
#include "sliceDataStorage.h" //For SliceLayerPart.

namespace cura
{

template<typename PathType>
const Polyline* PathOrdering<PathType>::getVertexData()
{
    return vertices_;
}

template<>
const Polyline* PathOrdering<const SkinPart*>::getVertexData()
{
    return &vertices_->outline.outerPolygon();
}

template<>
const Polyline* PathOrdering<const SliceLayerPart*>::getVertexData()
{
    return &vertices_->outline.outerPolygon();
}

template<>
const Polyline* PathOrdering<SliceLayerPart*>::getVertexData()
{
    return &vertices_->outline.outerPolygon();
}

template<>
const Polyline* PathOrdering<const SupportInfillPart*>::getVertexData()
{
    return &vertices_->outline_.outerPolygon();
}
template<>
const Polyline* PathOrdering<const ExtrusionLine*>::getVertexData()
{
    if (! cached_vertices_.has_value())
    {
        cached_vertices_ = vertices_->toPolygon();
    }
    return &(cached_vertices_.value());
}

template const Polyline* PathOrdering<Polygon*>::getVertexData();
template const Polyline* PathOrdering<Polygon const*>::getVertexData();
template const Polyline* PathOrdering<const OpenPolyline*>::getVertexData();
template const Polyline* PathOrdering<OpenPolyline*>::getVertexData();
template const Polyline* PathOrdering<ClosedPolyline*>::getVertexData();
template const Polyline* PathOrdering<Polyline const*>::getVertexData();

} // namespace cura
