// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/Coord_t.h"
#include "types/geometry/Point2LL.h"

namespace curaengine::types::geometry
{

class TransformedSegment;

/*! Helper structure that calculate the basic coordinates data to be used to perform segments overlapping calculation */
struct SegmentOverlappingData
{
    types::coord_t y_min;
    types::coord_t y_max;
    types::coord_t this_x_min;
    types::coord_t this_x_max;
    types::coord_t other_x_min;
    types::coord_t other_x_max;

    /*!
     * Builds the basic segment overlapping data
     * @param this_min_y The minimum Y coordinate of the original segment
     * @param this_max_y The maximum Y coordinate of the original segment
     * @param other_min_y The minimum Y coordinate of the tested segment
     * @param other_max_y The maximum Y coordinate of the tested segment
     * @param this_start The actual start position of the original segment, which may be null
     * @param this_end The actual end position of the original segment, which may be null
     * @param other_start The actual start position of the tested segment
     * @param other_end The actual end position of the tested segment
     */
    SegmentOverlappingData(
        const types::coord_t this_min_y,
        const types::coord_t this_max_y,
        const types::coord_t other_min_y,
        const types::coord_t other_max_y,
        const types::geometry::Point2LL* this_start,
        const types::geometry::Point2LL* this_end,
        const types::geometry::Point2LL& other_start,
        const types::geometry::Point2LL& other_end);

    /*! Make the cropped segment that is actually overlapping over the original segment, which may be part of all of the initial segment */
    TransformedSegment makeOtherOverlappingPart() const;
};

} // namespace curaengine::types::geometry
