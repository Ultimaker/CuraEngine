// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <utility>

#include "utils/PathsPointIndex.h"

namespace curaengine::utils
{

/*!
 * Locator to extract a line segment out of a \ref PolygonsPointIndex
 */
struct PolygonsPointIndexSegmentLocator
{
    std::pair<geometry::Point2LL, geometry::Point2LL> operator()(const PolygonsPointIndex& val) const;
};

} // namespace curaengine::utils
