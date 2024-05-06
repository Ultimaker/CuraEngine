// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MIXED_POLYLINE_STITCHER_H
#define UTILS_MIXED_POLYLINE_STITCHER_H

#include "PolylineStitcher.h"
#include "geometry/ClosedLinesSet.h"
#include "geometry/LinesSet.h"
#include "geometry/OpenLinesSet.h"

namespace cura
{

class MixedLinesSet;

class MixedPolylineStitcher : public PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>
{
public:
    static void stitch(const OpenLinesSet& lines, MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

} // namespace cura
#endif // UTILS_MIXED_POLYLINE_STITCHER_H
