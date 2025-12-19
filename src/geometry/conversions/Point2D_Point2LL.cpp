// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/conversions/Point2D_Point2LL.h"


namespace cura
{

Point2LL toPoint2LL(const Point2D& point)
{
    return Point2LL(std::llrint(point.x()), std::llrint(point.y()));
}

Point2D toPoint2D(const Point2LL& point)
{
    return Point2D(point.X, point.Y);
}

} // namespace cura
