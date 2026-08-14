// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT2F_H
#define POINT2F_H

#include <cmath>
#include <compare>

namespace cura
{

class Point2F
{
public:
    float x_{}, y_{};

    Point2F()
    {
    }

    Point2F(float x, float y)
        : x_(x)
        , y_(y)
    {
    }

    double vSize2() const
    {
        return x_ * x_ + y_ * y_;
    }

    double vSize() const
    {
        return std::sqrt(vSize2());
    }

    Point2F vNormalized() const
    {
        return *this / vSize();
    }

    Point2F operator/(const double scale) const
    {
        return Point2F(x_ / scale, y_ / scale);
    }

    auto operator<=>(const Point2F&) const = default;
};

static Point2F lerp(const Point2F& a, const Point2F& b, const float t)
{
    return Point2F(std::lerp(a.x_, b.x_, t), std::lerp(a.y_, b.y_, t));
}

} // namespace cura
#endif // POINT2F_H
