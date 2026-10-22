// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"
#include "geometry/LinesSet.h"
#include "geometry/Shape.h"

namespace cura
{

using OpenPolylineStitcher = PolylineStitcher<OpenLinesSet, Shape, OpenPolyline, Point2LL>;

} // namespace cura
