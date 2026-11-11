// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/Point2D.h"
#include "types/geometry/Point2LL.h"

namespace curaengine::types::geometry::conversions
{

Point2LL toPoint2LL(const Point2D& point);

Point2D toPoint2D(const Point2LL& point);

} // namespace curaengine::types::geometry::conversions
