// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace cura
{

class PointsSet;
class SliceDataStorage;
class Shape;

namespace seam
{
void calculateWallsSeamPositions(SliceDataStorage& storage);

size_t calculateShortestSeamPosition(const PointsSet& candidates, const Point2LL& target_pos, const Shape& seam_exclusion_areas);
} // namespace seam

} // namespace cura