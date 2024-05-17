// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT_MATRIX_H
#define GEOMETRY_POINT_MATRIX_H

#include <array>
#include <numbers>

#include "geometry/Point2LL.h"


namespace cura
{

class PointMatrix
{
public:
    std::array<double, 4> matrix{ 1, 0, 0, 1 };

    PointMatrix() noexcept = default;

    explicit PointMatrix(double rotation)
    {
        rotation = rotation / 180 * std::numbers::pi;
        matrix.at(0) = std::cos(rotation);
        matrix.at(1) = -std::sin(rotation);
        matrix.at(2) = -matrix.at(1);
        matrix.at(3) = matrix.at(0);
    }

    explicit PointMatrix(const Point2LL& p)
    {
        matrix.at(0) = static_cast<double>(p.X);
        matrix.at(1) = static_cast<double>(p.Y);
        double f = std::sqrt((matrix.at(0) * matrix.at(0)) + (matrix.at(1) * matrix.at(1)));
        matrix.at(0) /= f;
        matrix.at(1) /= f;
        matrix.at(2) = -matrix.at(1);
        matrix.at(3) = matrix.at(0);
    }

    static PointMatrix scale(double s)
    {
        PointMatrix ret;
        ret.matrix.at(0) = s;
        ret.matrix.at(3) = s;
        return ret;
    }

    [[nodiscard]] Point2LL apply(const Point2LL& p) const
    {
        const auto x = static_cast<double>(p.X);
        const auto y = static_cast<double>(p.Y);
        return { std::llrint(x * matrix.at(0) + y * matrix.at(1)), std::llrint(x * matrix.at(2) + y * matrix.at(3)) };
    }

    /*!
     * \warning only works on a rotation matrix! Output is incorrect for other types of matrix
     */
    [[nodiscard]] Point2LL unapply(const Point2LL& p) const
    {
        const auto x = static_cast<double>(p.X);
        const auto y = static_cast<double>(p.Y);
        return { std::llrint(x * matrix.at(0) + y * matrix.at(2)), std::llrint(x * matrix.at(1) + y * matrix.at(3)) };
    }

    [[nodiscard]] PointMatrix inverse() const
    {
        PointMatrix ret;
        double det = matrix.at(0) * matrix.at(3) - matrix.at(1) * matrix.at(2);
        ret.matrix.at(0) = matrix.at(3) / det;
        ret.matrix.at(1) = -matrix.at(1) / det;
        ret.matrix.at(2) = -matrix.at(2) / det;
        ret.matrix.at(3) = matrix.at(0) / det;
        return ret;
    }
};

} // namespace cura
#endif // GEOMETRY_POINT_MATRIX_H
