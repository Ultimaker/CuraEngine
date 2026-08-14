// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_SEGMENTOVERLAPPINGTYPE_H
#define BRIDGE_SEGMENTOVERLAPPINGTYPE_H

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

#endif
