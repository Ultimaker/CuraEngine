// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_OPEN_POLYLINE_STITCHER_H
#define UTILS_OPEN_POLYLINE_STITCHER_H

#include "PolylineStitcher.h"
#include "geometry/LinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"

namespace cura
{

using OpenPolylineStitcher = PolylineStitcher<OpenLinesSet, Shape, OpenPolyline, Point2LL>;

} // namespace cura
#endif // UTILS_OPEN_POLYLINE_STITCHER_H
