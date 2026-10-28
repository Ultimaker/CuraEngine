// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <array>

#include "geometry/Point2LL.h"

namespace curaengine::types
{
class AngleDegrees;
class AngleRadians;
} // namespace curaengine::types

namespace curaengine::geometry
{


class PointMatrix
{
public:
    std::array<double, 4> matrix{ 1, 0, 0, 1 };

    PointMatrix() noexcept = default;

    explicit PointMatrix(double rotation);

    explicit PointMatrix(const types::AngleDegrees& rotation);

    explicit PointMatrix(const types::AngleRadians& rotation);

    explicit PointMatrix(const Point2LL& p);

    static PointMatrix scale(double s);

    [[nodiscard]] Point2LL apply(const Point2LL& p) const;

    /*!
     * \warning only works on a rotation matrix! Output is incorrect for other types of matrix
     */
    [[nodiscard]] Point2LL unapply(const Point2LL& p) const;

    [[nodiscard]] PointMatrix inverse() const;
};

} // namespace curaengine::geometry
