// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/SegmentOverlappingType.h"
#include "types/geometry/TransformedSegment.h"

namespace curaengine::types::geometry
{

struct SegmentOverlapping
{
    SegmentOverlappingType type;
    TransformedSegment other_overlapping_part;
};

} // namespace curaengine::types::geometry
