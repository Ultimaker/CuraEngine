// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

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
