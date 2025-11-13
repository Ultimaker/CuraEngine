// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_CONVERSIONS_POINT2D_POINT2LL_H
#define GEOMETRY_CONVERSIONS_POINT2D_POINT2LL_H

#include "geometry/Point2D.h"
#include "geometry/Point2LL.h"

namespace cura
{

Point2LL toPoint2LL(const Point2D& point);

Point2D toPoint2D(const Point2LL& point);

} // namespace cura
#endif
