// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SlicedUVCoordinates.h"

#include "slicer.h"

namespace cura
{

SlicedUVCoordinates::SlicedUVCoordinates(const std::vector<SlicerSegment>& segments)
    : located_uv_coordinates_(cell_size)
{
    for (const SlicerSegment& segment : segments)
    {
        if (segment.uv_start.has_value() && segment.uv_end.has_value())
        {
            located_uv_coordinates_.insert(segment.start, segment.uv_start.value());
            located_uv_coordinates_.insert(segment.end, segment.uv_end.value());
        }
    }
}

std::optional<Point2F> SlicedUVCoordinates::getUVCoordinatesOfClosedPoint(const Point2LL& position) const
{
    SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Point2F> nearest_uv_coordinates;
    if (! located_uv_coordinates_.getNearest(position, search_radius, nearest_uv_coordinates))
    {
        return std::nullopt;
    }

    return nearest_uv_coordinates.val;
}

} // namespace cura
