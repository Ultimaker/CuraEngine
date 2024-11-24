// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT2D_H
#define GEOMETRY_POINT2D_H

#include <math.h>

#include "geometry/Point2LL.h"


namespace cura
{

/*
Double-precision 2D points are used for geometry computation.
*/
class Point2D
{
public:
    double x_{ 0.0 }, y_{ 0.0 };

    Point2D(double x = 0.0, double y = 0.0)
        : x_(x)
        , y_(y)
    {
    }

    Point2D(const Point2LL& point)
        : Point2D(static_cast<double>(point.X), static_cast<double>(point.Y))
    {
    }

    Point2D operator+(const Point2D& p) const
    {
        return Point2D(x_ + p.x_, y_ + p.y_);
    }
    Point2D operator-(const Point2D& p) const
    {
        return Point2D(x_ - p.x_, y_ - p.y_);
    }
    Point2D operator*(const double f) const
    {
        return Point2D(x_ * f, y_ * f);
    }
    Point2D operator/(const double f) const
    {
        return Point2D(x_ / f, y_ / f);
    }

    Point2D& operator+=(const Point2D& p)
    {
        x_ += p.x_;
        y_ += p.y_;
        return *this;
    }
    Point2D& operator-=(const Point2D& p)
    {
        x_ -= p.x_;
        y_ -= p.y_;
        return *this;
    }
    Point2D& operator*=(const double f)
    {
        x_ *= f;
        y_ *= f;
        return *this;
    }

    bool operator==(Point2D& p) const
    {
        return x_ == p.x_ && y_ == p.y_;
    }
    bool operator!=(Point2D& p) const
    {
        return x_ != p.x_ || y_ != p.y_;
    }

    double vSize2() const
    {
        return x_ * x_ + y_ * y_;
    }

    double vSize() const
    {
        return std::sqrt(vSize2());
    }

    Point2D normalized() const
    {
        const double size = vSize();
        return size != 0.0 ? ((*this) / size) : Point2D();
    }

    Point2D turned90CCW() const
    {
        return { -y_, x_ };
    }

    double dot(const Point2D& other) const
    {
        return x_ * other.x_ + y_ * other.y_;
    }
};

} // namespace cura
#endif // GEOMETRY_POINT2D_H
