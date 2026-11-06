// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/Point2LL.h"

#include "types/geometry/Point3LL.h"

namespace curaengine::types::geometry
{

Point2LL operator+(const Point2LL& p2, const Point3LL& p3)
{
    return { p3.x_ + p2.X, p3.y_ + p2.Y };
}

Point3LL operator+(const Point3LL& p3, const Point2LL& p2)
{
    return { p3.x_ + p2.X, p3.y_ + p2.Y, p3.z_ };
}

Point3LL& operator+=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ += p2.X;
    p3.y_ += p2.Y;
    return p3;
}

Point3LL operator-(const Point3LL& p3, const Point2LL& p2)
{
    return { p3.x_ - p2.X, p3.y_ - p2.Y, p3.z_ };
}

Point3LL& operator-=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ -= p2.X;
    p3.y_ -= p2.Y;
    return p3;
}

Point2LL operator-(const Point2LL& p2, const Point3LL& p3)
{
    return { p2.X - p3.x_, p2.Y - p3.y_ };
}

} // namespace curaengine::types::geometry
