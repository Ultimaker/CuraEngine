// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT_MATRIX_H
#define GEOMETRY_POINT_MATRIX_H

#include <array>

#include "geometry/Point2LL.h"


namespace cura
{

class AngleDegrees;
class AngleRadians;

class PointMatrix
{
public:
    std::array<double, 4> matrix{ 1, 0, 0, 1 };

    PointMatrix() noexcept = default;

    explicit PointMatrix(double rotation);

    explicit PointMatrix(const AngleDegrees& rotation);

    explicit PointMatrix(const AngleRadians& rotation);

    explicit PointMatrix(const Point2LL& p);

    static PointMatrix scale(double s);

    [[nodiscard]] Point2LL apply(const Point2LL& p) const;

    /*!
     * \warning only works on a rotation matrix! Output is incorrect for other types of matrix
     */
    [[nodiscard]] Point2LL unapply(const Point2LL& p) const;

    [[nodiscard]] PointMatrix inverse() const;
};

} // namespace cura
#endif // GEOMETRY_POINT_MATRIX_H
