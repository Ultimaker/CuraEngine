// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "path_ordering.h" //The definitions we're implementing here.

#include "WallToolPaths.h"
#include "geometry/OpenPolyline.h"
#include "sliceDataStorage.h" //For SliceLayerPart.

namespace cura
{

template<typename PathType>
const PointsSet& PathOrdering<PathType>::getVertexData()
{
    return *vertices_;
}

template<>
const PointsSet& PathOrdering<const SkinPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const PointsSet& PathOrdering<const SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const PointsSet& PathOrdering<SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const PointsSet& PathOrdering<const SupportInfillPart*>::getVertexData()
{
    return vertices_->outline_.outerPolygon();
}
template<>
const PointsSet& PathOrdering<const ExtrusionLine*>::getVertexData()
{
    if (! cached_vertices_)
    {
        cached_vertices_ = vertices_->toPolygon();
    }
    return *cached_vertices_;
}

template const PointsSet& PathOrdering<Polygon*>::getVertexData();
template const PointsSet& PathOrdering<Polygon const*>::getVertexData();
template const PointsSet& PathOrdering<const OpenPolyline*>::getVertexData();
template const PointsSet& PathOrdering<OpenPolyline*>::getVertexData();
template const PointsSet& PathOrdering<ClosedPolyline*>::getVertexData();
template const PointsSet& PathOrdering<Polyline const*>::getVertexData();

} // namespace cura
