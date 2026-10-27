// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace cura
{

/*! Enumeration containing the overlapping type of two segments in the bridging projection direction */
enum class SegmentOverlappingType
{
    Full, // The segment fully overlaps with the base segment
    Bottom, // The segment overlaps with the base segment only on its bottom part
    Top, // The segment overlaps with the base segment only on its top part
    Middle, // The segment overlaps with the base segment somewhere in the middle, but not top neither bottom
};

} // namespace cura
