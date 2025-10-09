// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT3D_H
#define POINT3D_H

#include <compare>
#include <math.h>
#include <stdint.h>

#include "geometry/Point3LL.h"


namespace cura
{

/*
Double-precision 3D points are used for geometry computation.
*/
class Point3D
{
public:
    double x_, y_, z_;

    Point3D()
    {
    }

    Point3D(double x, double y, double z)
        : x_(x)
        , y_(y)
        , z_(z)
    {
    }

    explicit Point3D(const Point3LL& p, const double scale = 0.001)
        : x_(static_cast<double>(p.x_) * scale)
        , y_(static_cast<double>(p.y_) * scale)
        , z_(static_cast<double>(p.z_) * scale)
    {
    }

    Point3D& operator+=(const Point3D& p)
    {
        x_ += p.x_;
        y_ += p.y_;
        z_ += p.z_;
        return *this;
    }
    Point3D& operator-=(const Point3D& p)
    {
        x_ -= p.x_;
        y_ -= p.y_;
        z_ -= p.z_;
        return *this;
    }
    Point3D& operator*=(const double f)
    {
        x_ *= f;
        y_ *= f;
        z_ *= f;
        return *this;
    }

    bool operator==(const Point3D& p) const
    {
        return x_ == p.x_ && y_ == p.y_ && z_ == p.z_;
    }
    bool operator!=(Point3D& p) const
    {
        return x_ != p.x_ || y_ != p.y_ || z_ != p.z_;
    }

    auto operator<=>(const Point3D&) const = default;

    Point3D operator+(const Point3D& other) const
    {
        return Point3D(x_ + other.x_, y_ + other.y_, z_ + other.z_);
    }

    Point3D operator/(const double divisor) const
    {
        return Point3D(x_ / divisor, y_ / divisor, z_ / divisor);
    }

    Point3D operator*(const double factor) const
    {
        return Point3D(x_ * factor, y_ * factor, z_ * factor);
    }

    double operator*(const Point3D& other) const
    {
        return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
    }

    Point3D operator-(const Point3D& other) const
    {
        return Point3D(x_ - other.x_, y_ - other.y_, z_ - other.z_);
    }

    double max() const
    {
        if (x_ > y_ && x_ > z_)
            return x_;
        if (y_ > z_)
            return y_;
        return z_;
    }

    bool testLength(double len) const
    {
        return vSize2() <= len * len;
    }

    double vSize2() const
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }

    double vSize() const
    {
        return sqrt(vSize2());
    }

    inline Point3D normalized() const
    {
        return (*this) / vSize();
    }

    Point3D cross(const Point3D& p) const
    {
        return Point3D(y_ * p.z_ - z_ * p.y_, z_ * p.x_ - x_ * p.z_, x_ * p.y_ - y_ * p.x_);
    }

    static Point3D cross(const Point3LL& a, const Point3LL& b)
    {
        return Point3D(a).cross(Point3D(b));
    }

    Point3LL toPoint3()
    {
        return Point3LL(MM2INT(x_), MM2INT(y_), MM2INT(z_));
    }
};

} // namespace cura
#endif // POINT3D_H
