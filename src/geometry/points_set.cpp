// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/points_set.h"

#include "geometry/point2ll.h"
#include "geometry/point3_matrix.h"
#include "geometry/point_matrix.h"

namespace cura
{

PointsSet::PointsSet(const std::initializer_list<Point2LL>& initializer)
    : points_(initializer)
{
}

PointsSet::PointsSet(const ClipperLib::Path& points)
    : points_(points)
{
}

PointsSet::PointsSet(ClipperLib::Path&& points)
    : points_(std::move(points))
{
}

void PointsSet::applyMatrix(const PointMatrix& matrix)
{
    for (Point2LL& point : points_)
    {
        point = matrix.apply(point);
    }
}

void PointsSet::applyMatrix(const Point3Matrix& matrix)
{
    for (Point2LL& point : points_)
    {
        point = matrix.apply(point);
    }
}

Point2LL PointsSet::min() const
{
    Point2LL ret = Point2LL(POINT_MAX, POINT_MAX);
    for (Point2LL point : points_)
    {
        ret.X = std::min(ret.X, point.X);
        ret.Y = std::min(ret.Y, point.Y);
    }
    return ret;
}

Point2LL PointsSet::max() const
{
    Point2LL ret = Point2LL(POINT_MIN, POINT_MIN);
    for (Point2LL point : points_)
    {
        ret.X = std::max(ret.X, point.X);
        ret.Y = std::max(ret.Y, point.Y);
    }
    return ret;
}

Point2LL PointsSet::closestPointTo(const Point2LL& p) const
{
    const Point2LL* ret = &p;
    double bestDist = std::numeric_limits<double>::max();
    for (const Point2LL& point : points_)
    {
        double dist = vSize2f(p - point);
        if (dist < bestDist)
        {
            ret = &point;
            bestDist = dist;
        }
    }
    return *ret;
}

void PointsSet::translate(const Point2LL& translation)
{
    for (Point2LL& point : points_)
    {
        point += translation;
    }
}

} // namespace cura
