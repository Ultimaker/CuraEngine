// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/conversions/Point2D_Point2LL.h"


namespace curaengine::types::geometry::conversions
{

Point2LL toPoint2LL(const Point2D& point)
{
    return Point2LL(std::llrint(point.x()), std::llrint(point.y()));
}

Point2D toPoint2D(const Point2LL& point)
{
    return Point2D(point.X, point.Y);
}

} // namespace curaengine::types::geometry::conversions
