// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_SEGMENTOVERLAPPING_H
#define BRIDGE_SEGMENTOVERLAPPING_H

#include "bridge/SegmentOverlappingType.h"
#include "bridge/TransformedSegment.h"

namespace cura
{

struct SegmentOverlapping
{
    SegmentOverlappingType type;
    TransformedSegment other_overlapping_part;
};

} // namespace cura

#endif
