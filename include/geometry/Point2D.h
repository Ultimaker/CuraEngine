// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POINT2D_H
#define UTILS_POINT2D_H

#include "geometry/Point2LL.h"

namespace cura
{

class Point2D
{
private:
    double x_{ 0.0 };
    double y_{ 0.0 };

public:
    Point2D(double x, double y);

    Point2D(const Point2LL& other);

    [[nodiscard]] Point2D operator*(const double scale) const;

    [[nodiscard]] Point2D operator/(const double scale) const;

    [[nodiscard]] double getX() const
    {
        return x_;
    }

    [[nodiscard]] double getY() const
    {
        return y_;
    }

    [[nodiscard]] double size() const;

    [[nodiscard]] double size2() const;

    [[nodiscard]] Point2D normalized() const;

    void normalize();

    Point2LL toPoint2LL() const;
};

} // namespace cura

#endif // UTILS_POINT2D_H
