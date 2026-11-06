// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/PointsSet.h"

#include "types/geometry/Point3Matrix.h"
#include "types/geometry/PointMatrix.h"

namespace curaengine::types::geometry
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

} // namespace curaengine::types::geometry
