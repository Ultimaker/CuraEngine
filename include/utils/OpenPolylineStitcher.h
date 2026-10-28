// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"
#include "geometry/LinesSet.h"
#include "geometry/Shape.h"

namespace curaengine::utils
{

using OpenPolylineStitcher = PolylineStitcher<geometry::OpenLinesSet, geometry::Shape, geometry::OpenPolyline, geometry::Point2LL>;

} // namespace curaengine::utils
