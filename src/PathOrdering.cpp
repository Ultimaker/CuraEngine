// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrdering.h" //The definitions we're implementing here.

#include "SkinPart.h"
#include "SliceDataStorage.h" //For SliceLayerPart.
#include "SliceLayerPart.h"
#include "WallToolPaths.h"
#include "geometry/OpenPolyline.h"

namespace curaengine
{

template<typename PathType>
const ::curaengine::geometry::Polyline& PathOrdering<PathType>::getVertexData()
{
    return *vertices_;
}

template<>
const ::curaengine::geometry::Polyline& PathOrdering<const SkinPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const ::curaengine::geometry::Polyline& PathOrdering<const SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const ::curaengine::geometry::Polyline& PathOrdering<SliceLayerPart*>::getVertexData()
{
    return vertices_->outline.outerPolygon();
}

template<>
const ::curaengine::geometry::Polyline& PathOrdering<const SupportInfillPart*>::getVertexData()
{
    return vertices_->outline_.outerPolygon();
}
template<>
const ::curaengine::geometry::Polyline& PathOrdering<const ::curaengine::utils::ExtrusionLine*>::getVertexData()
{
    if (! cached_vertices_.has_value())
    {
        cached_vertices_ = vertices_->toPolygon();
    }
    return *cached_vertices_;
}

template const ::curaengine::geometry::Polyline& PathOrdering<::curaengine::geometry::Polygon*>::getVertexData();
template const ::curaengine::geometry::Polyline& PathOrdering<::curaengine::geometry::Polygon const*>::getVertexData();
template const ::curaengine::geometry::Polyline& PathOrdering<const ::curaengine::geometry::OpenPolyline*>::getVertexData();
template const ::curaengine::geometry::Polyline& PathOrdering<::curaengine::geometry::OpenPolyline*>::getVertexData();
template const ::curaengine::geometry::Polyline& PathOrdering<::curaengine::geometry::ClosedPolyline*>::getVertexData();
template const ::curaengine::geometry::Polyline& PathOrdering<::curaengine::geometry::Polyline const*>::getVertexData();

} // namespace curaengine
