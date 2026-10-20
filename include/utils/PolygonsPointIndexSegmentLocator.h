// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <utility>

#include "utils/PathsPointIndex.h"

namespace cura
{

/*!
 * Locator to extract a line segment out of a \ref PolygonsPointIndex
 */
struct PolygonsPointIndexSegmentLocator
{
    std::pair<Point2LL, Point2LL> operator()(const PolygonsPointIndex& val) const;
};

} // namespace cura
