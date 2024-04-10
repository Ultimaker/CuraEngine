// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MIXED_POLYLINE_STITCHER_H
#define UTILS_MIXED_POLYLINE_STITCHER_H

#include "PolylineStitcher.h"
#include "geometry/closed_lines_set.h"
#include "geometry/open_lines_set.h"

namespace cura
{

class MixedLinesSet;

class MixedPolylineStitcher : public PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>
{
public:
    static void stitch(const LinesSet<OpenPolyline>& lines, MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

} // namespace cura
#endif // UTILS_MIXED_POLYLINE_STITCHER_H
