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






class MixedPolylineStitcher : public PolylineStitcher<::curaengine::geometry::OpenLinesSet, ::curaengine::geometry::ClosedLinesSet, ::curaengine::geometry::OpenPolyline, ::curaengine::geometry::Point2LL>
{
public:
    static void stitch(const ::curaengine::geometry::OpenLinesSet& lines, ::curaengine::geometry::MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

} // namespace curaengine::utils
