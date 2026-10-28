// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"

namespace curaengine
{

class MixedLinesSet;
class OpenPolyline;
class OpenLinesSet;
class ClosedLinesSet;

class MixedPolylineStitcher : public PolylineStitcher<OpenLinesSet, ClosedLinesSet, OpenPolyline, Point2LL>
{
public:
    static void stitch(const OpenLinesSet& lines, MixedLinesSet& result, coord_t max_stitch_distance = MM2INT(0.1), coord_t snap_distance = 10);
};

} // namespace curaengine
