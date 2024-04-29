// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/PointsSet.h"

#include "geometry/Point3Matrix.h"
#include "geometry/PointMatrix.h"

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

void PointsSet::translate(const Point2LL& translation)
{
    for (Point2LL& point : points_)
    {
        point += translation;
    }
}

} // namespace cura
