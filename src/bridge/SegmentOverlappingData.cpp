// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/SegmentOverlappingData.h"

#include "bridge/TransformedSegment.h"
#include "utils/linearAlg2D.h"


namespace cura
{

SegmentOverlappingData::SegmentOverlappingData(
    const coord_t this_min_y,
    const coord_t this_max_y,
    const coord_t other_min_y,
    const coord_t other_max_y,
    const Point2LL* this_start,
    const Point2LL* this_end,
    const Point2LL& other_start,
    const Point2LL& other_end)
    : y_min(std::max(this_min_y, other_min_y))
    , y_max(std::min(this_max_y, other_max_y))
    , this_x_min(this_start != nullptr ? LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_min).value() : 0)
    , this_x_max(this_start != nullptr ? LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_max).value() : 0)
    , other_x_min(LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_min).value())
    , other_x_max(LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_max).value())
{
}

/*! Make the cropped segment that is actually overlapping over the original segment, which may be part of all of the initial segment */
TransformedSegment SegmentOverlappingData::makeOtherOverlappingPart() const
{
    return TransformedSegment(Point2LL(other_x_min, y_min), Point2LL(other_x_max, y_max));
}

} // namespace cura
