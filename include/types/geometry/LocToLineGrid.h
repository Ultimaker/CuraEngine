// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/PolygonsPointIndexSegmentLocator.h"
#include "types/geometry/SparseLineGrid.h"

namespace curaengine::types::geometry
{

using LocToLineGrid = SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator>;

} // namespace curaengine::types::geometry
