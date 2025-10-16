// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT2D_H
#define POINT2D_H

#include <cmath>

namespace cura
{

class Point2D
{
public:
    Point2D() = default;

    Point2D(double x, double y)
        : x_(x)
        , y_(y)
    {
    }

    double x() const
    {
        return x_;
    }

    double y() const
    {
        return y_;
    }

    double vSize2() const
    {
        return x_ * x_ + y_ * y_;
    }

    double vSize() const
    {
        return std::sqrt(vSize2());
    }

    Point2D vNormalized() const
    {
        return *this / vSize();
    }

    Point2D rotated90CCW() const
    {
        return Point2D(-y_, x_);
    }

    Point2D operator/(const double scale) const
    {
        return Point2D(x_ / scale, y_ / scale);
    }

    static double dot(const Point2D& p0, const Point2D& p1)
    {
        return p0.x_ * p1.x_ + p0.y_ * p1.y_;
    }

    auto operator<=>(const Point2D&) const = default;

private:
    double x_{}, y_{};
};

} // namespace cura
#endif // POINT2D_H
