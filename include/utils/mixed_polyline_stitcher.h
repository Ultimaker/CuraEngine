// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_OPEN_POLYLINE_STITCHER_H
#define UTILS_OPEN_POLYLINE_STITCHER_H

#include "PolylineStitcher.h"
#include "geometry/closed_polyline.h"
#include "geometry/mixed_lines_set.h"
#include "geometry/open_polyline.h"

namespace cura
{

class MixedPolylineStitcher : public PolylineStitcher<LinesSet<OpenPolyline>, LinesSet<ClosedPolyline>, OpenPolyline, Point2LL>
{
public:
    static void stitch(const LinesSet<OpenPolyline>& lines, MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10)
    {
        PolylineStitcher::stitch(lines, result.getOpenLines(), result.getClosedLines(), max_stitch_distance, snap_distance);
    }
};

} // namespace cura
#endif // UTILS_OPEN_POLYLINE_STITCHER_H
