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

class MixedPolylineStitcher : public PolylineStitcher<LinesSet<Polyline>, LinesSet<Polyline>, Polyline, Point2LL>
{
public:
    static void stitch(const LinesSet<Polyline>& lines, MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10)
    {
        LinesSet<Polyline> open_lines;
        LinesSet<Polyline> closed_lines;

        PolylineStitcher::stitch(lines, open_lines, closed_lines, max_stitch_distance, snap_distance);

        result.push_back(std::move(open_lines));

        for (Polyline& closed_line : closed_lines)
        {
            // Base stitch method will create explicitely closed polylines, but won't tag them as such
            // because it is a generic algporitm. Tag them now.
            closed_line.setType(PolylineType::ExplicitelyClosed);
        }

        result.push_back(std::move(closed_lines));
    }
};

} // namespace cura
#endif // UTILS_OPEN_POLYLINE_STITCHER_H
