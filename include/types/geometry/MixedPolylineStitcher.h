// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/PolylineStitcher.h"

namespace curaengine::types::geometry
{
class MixedLinesSet;
class OpenPolyline;
class OpenLinesSet;
class ClosedLinesSet;
} // namespace curaengine::types::geometry

namespace curaengine::types::geometry
{


class MixedPolylineStitcher : public PolylineStitcher<types::geometry::OpenLinesSet, types::geometry::ClosedLinesSet, types::geometry::OpenPolyline, types::geometry::Point2LL>
{
public:
    static void stitch(const types::geometry::OpenLinesSet& lines, types::geometry::MixedLinesSet& result, coord_t max_stitch_distance = mmToInt(0.1), coord_t snap_distance = 10);
};

} // namespace curaengine::types::geometry
