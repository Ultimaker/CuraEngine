// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/mixed_polyline_stitcher.h"

#include "geometry/mixed_lines_set.h"
#include "geometry/shape.h"


namespace cura
{

void MixedPolylineStitcher::stitch(const LinesSet<OpenPolyline>& lines, MixedLinesSet& result, coord_t max_stitch_distance, coord_t snap_distance)
{
    OpenLinesSet open_lines;
    ClosedLinesSet closed_lines;

    PolylineStitcher::stitch(lines, open_lines, closed_lines, max_stitch_distance, snap_distance);

    result.push_back(std::move(open_lines));

    for (ClosedPolyline& closed_line : closed_lines)
    {
        // Base stitch method will create explicitely closed polylines, but won't tag them as such
        // because it is a generic algorithm. Tag them now.
        closed_line.setExplicitelyClosed(true);
    }

    result.push_back(std::move(closed_lines));
}

} // namespace cura
