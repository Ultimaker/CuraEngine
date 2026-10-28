// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2D.h"
#include "geometry/Point2LL.h"

namespace curaengine
{

Point2LL toPoint2LL(const Point2D& point);

Point2D toPoint2D(const Point2LL& point);

} // namespace curaengine
