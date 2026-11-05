// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <utility>

#include "types/geometry/PathsPointIndex.h"

namespace curaengine::types::geometry
{

/*!
 * Locator to extract a line segment out of a \ref PolygonsPointIndex
 */
struct PolygonsPointIndexSegmentLocator
{
    std::pair<types::geometry::Point2LL, types::geometry::Point2LL> operator()(const PolygonsPointIndex& val) const;
};

} // namespace curaengine::types::geometry
