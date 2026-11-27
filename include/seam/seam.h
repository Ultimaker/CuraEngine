// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/Point2LL.h"

namespace curaengine
{

class SliceDataStorage;

namespace types::geometry
{
class PointsSet;
class Shape;
} // namespace types::geometry

namespace seam
{
void calculateWallsSeamPositions(SliceDataStorage& storage);

size_t calculateShortestSeamPosition(const types::geometry::PointsSet& candidates, const types::geometry::Point2LL& target_pos, const types::geometry::Shape& seam_exclusion_areas);
} // namespace seam

} // namespace curaengine