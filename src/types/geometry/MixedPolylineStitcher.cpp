// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/MixedPolylineStitcher.h"

#include "types/geometry/ClosedLinesSet.h"
#include "types/geometry/MixedLinesSet.h"
#include "types/geometry/OpenLinesSet.h"
#include "types/geometry/Polygon.h"
#include "types/geometry/Shape.h"


namespace curaengine::types::geometry
{

using namespace types;
using namespace utils;

void MixedPolylineStitcher::stitch(const OpenLinesSet& lines, MixedLinesSet& result, coord_t max_stitch_distance, coord_t snap_distance)
{
    OpenLinesSet open_lines;
    ClosedLinesSet closed_lines;

    PolylineStitcher::stitch(lines, open_lines, closed_lines, max_stitch_distance, snap_distance);

    result.push_back(std::move(open_lines));

    for (ClosedPolyline& closed_line : closed_lines)
    {
        // Base stitch method will create explicitly closed polylines, but won't tag them as such
        // because it is a generic algorithm. Tag them now.
        closed_line.setExplicitlyClosed(true);
    }

    result.push_back(std::move(closed_lines));
}

} // namespace curaengine::types::geometry
