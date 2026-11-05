// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/ExpansionRange.h"

#include "types/geometry/SegmentOverlapping.h"
#include "types/geometry/SegmentOverlappingData.h"


namespace curaengine::types::geometry
{
using namespace types;
using namespace utils;

std::optional<SegmentOverlapping> ExpansionRange::calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const
{
    if (is_projected_)
    {
        return data_.segment.calculateOverlapping(other, expand_direction);
    }

    if (fuzzy_is_greater_or_equal(other.minY(), maxY()) || fuzzy_is_lesser_or_equal(other.maxY(), minY()))
    {
        // Not on the same horizontal band, or very slightly overlapping , discard
        return std::nullopt;
    }

    const SegmentOverlappingData overlapping(data_.range.min_y, data_.range.max_y, other.minY(), other.maxY(), nullptr, nullptr, other.getStart(), other.getEnd());

    const bool overlap_top = (other.maxY() == data_.range.max_y);
    const bool overlap_bottom = (other.minY() == data_.range.min_y);

    return SegmentOverlapping{ TransformedSegment::makeNonIntersectingOverlapping(overlap_top, overlap_bottom), overlapping.makeOtherOverlappingPart() };
}

} // namespace curaengine::types::geometry
