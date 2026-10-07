// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/PointMatrix.h"

#include <numbers>

#include "settings/types/Angle.h"


namespace cura
{

PointMatrix::PointMatrix(double rotation)
{
    rotation = rotation / 180 * std::numbers::pi;
    matrix[0] = std::cos(rotation);
    matrix[1] = -std::sin(rotation);
    matrix[2] = -matrix[1];
    matrix[3] = matrix[0];
}

PointMatrix::PointMatrix(const AngleDegrees& rotation)
    : PointMatrix(rotation.value_)
{
}

PointMatrix::PointMatrix(const AngleRadians& rotation)
    : PointMatrix(AngleDegrees(rotation))
{
}

PointMatrix::PointMatrix(const Point2LL& p)
{
    matrix[0] = static_cast<double>(p.X);
    matrix[1] = static_cast<double>(p.Y);
    double f = std::sqrt((matrix[0] * matrix[0]) + (matrix[1] * matrix[1]));
    matrix[0] /= f;
    matrix[1] /= f;
    matrix[2] = -matrix[1];
    matrix[3] = matrix[0];
}

PointMatrix PointMatrix::scale(double s)
{
    PointMatrix ret;
    ret.matrix[0] = s;
    ret.matrix[3] = s;
    return ret;
}

Point2LL PointMatrix::apply(const Point2LL& p) const
{
    const auto x = static_cast<double>(p.X);
    const auto y = static_cast<double>(p.Y);
    return { std::llrint(x * matrix[0] + y * matrix[1]), std::llrint(x * matrix[2] + y * matrix[3]) };
}

Point2LL PointMatrix::unapply(const Point2LL& p) const
{
    const auto x = static_cast<double>(p.X);
    const auto y = static_cast<double>(p.Y);
    return { std::llrint(x * matrix[0] + y * matrix[2]), std::llrint(x * matrix[1] + y * matrix[3]) };
}

PointMatrix PointMatrix::inverse() const
{
    PointMatrix ret;
    double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    ret.matrix[0] = matrix[3] / det;
    ret.matrix[1] = -matrix[1] / det;
    ret.matrix[2] = -matrix[2] / det;
    ret.matrix[3] = matrix[0] / det;
    return ret;
}

} // namespace cura
