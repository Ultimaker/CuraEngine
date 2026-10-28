// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "bridge/SegmentOverlappingType.h"
#include "bridge/TransformedSegment.h"

namespace curaengine::bridge
{

struct SegmentOverlapping
{
    SegmentOverlappingType type;
    TransformedSegment other_overlapping_part;
};

} // namespace curaengine::bridge
