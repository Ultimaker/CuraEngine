// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_SEGMENTOVERLAPPINGDATA_H
#define BRIDGE_SEGMENTOVERLAPPINGDATA_H

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

class TransformedSegment;

/*! Helper structure that calculate the basic coordinates data to be used to perform segments overlapping calculation */
struct SegmentOverlappingData
{
    coord_t y_min;
    coord_t y_max;
    coord_t this_x_min;
    coord_t this_x_max;
    coord_t other_x_min;
    coord_t other_x_max;

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
        const coord_t this_min_y,
        const coord_t this_max_y,
        const coord_t other_min_y,
        const coord_t other_max_y,
        const Point2LL* this_start,
        const Point2LL* this_end,
        const Point2LL& other_start,
        const Point2LL& other_end);

    /*! Make the cropped segment that is actually overlapping over the original segment, which may be part of all of the initial segment */
    TransformedSegment makeOtherOverlappingPart() const;
};

} // namespace cura

#endif
