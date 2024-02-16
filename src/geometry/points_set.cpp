// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/points_set.h"

#include "geometry/point2ll.h"
#include "geometry/point3_matrix.h"
#include "geometry/point_matrix.h"

namespace cura
{

PointsSet::PointsSet(const std::initializer_list<Point2LL>& initializer)
    : std::vector<Point2LL>(initializer)
{
}

PointsSet::PointsSet(const std::vector<Point2LL>& points)
    : std::vector<Point2LL>(points)
{
}

PointsSet::PointsSet(std::vector<Point2LL>&& points)
    : std::vector<Point2LL>(std::move(points))
{
}

void PointsSet::applyMatrix(const PointMatrix& matrix)
{
    for (Point2LL& point : (*this))
    {
        point = matrix.apply(point);
    }
}

void PointsSet::applyMatrix(const Point3Matrix& matrix)
{
    for (Point2LL& point : (*this))
    {
        point = matrix.apply(point);
    }
}

Point2LL PointsSet::min() const
{
    Point2LL ret = Point2LL(POINT_MAX, POINT_MAX);
    for (Point2LL p : *this)
    {
        ret.X = std::min(ret.X, p.X);
        ret.Y = std::min(ret.Y, p.Y);
    }
    return ret;
}

Point2LL PointsSet::max() const
{
    Point2LL ret = Point2LL(POINT_MIN, POINT_MIN);
    for (Point2LL p : *this)
    {
        ret.X = std::max(ret.X, p.X);
        ret.Y = std::max(ret.Y, p.Y);
    }
    return ret;
}

Point2LL PointsSet::closestPointTo(const Point2LL& p) const
{
    Point2LL ret = p;
    double bestDist = std::numeric_limits<double>::max();
    for (size_t n = 0; n < size(); n++)
    {
        double dist = vSize2f(p - (*this)[n]);
        if (dist < bestDist)
        {
            ret = (*this)[n];
            bestDist = dist;
        }
    }
    return ret;
}

void PointsSet::translate(const Point2LL& translation)
{
    for (Point2LL& p : *this)
    {
        p += translation;
    }
}

} // namespace cura
