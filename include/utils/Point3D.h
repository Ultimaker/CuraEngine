// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT3D_H
#define POINT3D_H

#include <math.h>
#include <stdint.h>

#include "IntPoint.h"


namespace cura
{

/*
Double-precision 3D points are used for geometry computation.
They represent millimeters in 3D space.
*/
class Point3D
{
public:
    double x, y, z;

    Point3D()
    {
    }

    Point3D(double _x, double _y, double _z)
        : x(_x)
        , y(_y)
        , z(_z)
    {
    }

    Point3D(const Point3& p)
        : x(static_cast<double>(p.x_) * .001)
        , y(static_cast<double>(p.y_) * .001)
        , z(static_cast<double>(p.z_) * .001)
    {
    }

    Point3D operator+(const Point3D& p) const
    {
        return Point3D(x + p.x, y + p.y, z + p.z);
    }
    Point3D operator-(const Point3D& p) const
    {
        return Point3D(x - p.x, y - p.y, z - p.z);
    }
    Point3D operator*(const double f) const
    {
        return Point3D(x * f, y * f, z * f);
    }
    Point3D operator/(const double f) const
    {
        return Point3D(x / f, y / f, z / f);
    }

    Point3D& operator+=(const Point3D& p)
    {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }
    Point3D& operator-=(const Point3D& p)
    {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return *this;
    }
    Point3D& operator*=(const double f)
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }

    bool operator==(Point3D& p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(Point3D& p) const
    {
        return x != p.x || y != p.y || z != p.z;
    }

    double max() const
    {
        if (x > y && x > z)
            return x;
        if (y > z)
            return y;
        return z;
    }

    bool testLength(double len) const
    {
        return vSize2() <= len * len;
    }

    double vSize2() const
    {
        return x * x + y * y + z * z;
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
        return Point3D(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }

    static Point3D cross(const Point3& a, const Point3& b)
    {
        return Point3D(a).cross(Point3D(b));
    }

    Point3 toPoint3()
    {
        return Point3(MM2INT(x), MM2INT(y), MM2INT(z));
    }
};

inline double operator*(Point3D lhs, const Point3D& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

} // namespace cura
#endif // POINT3D_H
