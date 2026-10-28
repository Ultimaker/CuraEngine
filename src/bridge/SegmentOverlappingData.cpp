// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/SegmentOverlappingData.h"

#include "bridge/TransformedSegment.h"
#include "utils/LinearAlg2D.h"


namespace curaengine::bridge
{

SegmentOverlappingData::SegmentOverlappingData(
    const ::curaengine::utils::coord_t this_min_y,
    const ::curaengine::utils::coord_t this_max_y,
    const ::curaengine::utils::coord_t other_min_y,
    const ::curaengine::utils::coord_t other_max_y,
    const ::curaengine::geometry::Point2LL* this_start,
    const ::curaengine::geometry::Point2LL* this_end,
    const ::curaengine::geometry::Point2LL& other_start,
    const ::curaengine::geometry::Point2LL& other_end)
    : y_min(std::max(this_min_y, other_min_y))
    , y_max(std::min(this_max_y, other_max_y))
    , this_x_min(this_start != nullptr ? ::curaengine::utils::LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_min).value() : 0)
    , this_x_max(this_start != nullptr ? ::curaengine::utils::LinearAlg2D::lineHorizontalLineIntersection(*this_start, *this_end, y_max).value() : 0)
    , other_x_min(::curaengine::utils::LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_min).value())
    , other_x_max(::curaengine::utils::LinearAlg2D::lineHorizontalLineIntersection(other_start, other_end, y_max).value())
{
}

/*! Make the cropped segment that is actually overlapping over the original segment, which may be part of all of the initial segment */
TransformedSegment SegmentOverlappingData::makeOtherOverlappingPart() const
{
    return TransformedSegment(::curaengine::geometry::Point2LL(other_x_min, y_min), ::curaengine::geometry::Point2LL(other_x_max, y_max));
}

} // namespace curaengine::bridge
