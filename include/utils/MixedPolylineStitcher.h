// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"

namespace curaengine::geometry
{
class MixedLinesSet;
class OpenPolyline;
class OpenLinesSet;
class ClosedLinesSet;
}

namespace curaengine::utils
{






class MixedPolylineStitcher : public PolylineStitcher<geometry::OpenLinesSet, geometry::ClosedLinesSet, geometry::OpenPolyline, geometry::Point2LL>
{
public:
    static void stitch(const geometry::OpenLinesSet& lines, geometry::MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

} // namespace curaengine::utils
