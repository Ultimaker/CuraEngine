// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/LinesSet.h"
#include "types/geometry/PolylineStitcher.h"
#include "types/geometry/Shape.h"

namespace curaengine::types::geometry
{

using OpenPolylineStitcher = PolylineStitcher<types::geometry::OpenLinesSet, types::geometry::Shape, types::geometry::OpenPolyline, types::geometry::Point2LL>;

} // namespace curaengine::types::geometry
