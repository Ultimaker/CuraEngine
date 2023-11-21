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
class Point3d
{
public:
    double x, y, z;

    Point3d()
    {
    }

    Point3d(double _x, double _y, double _z)
        : x(_x)
        , y(_y)
        , z(_z)
    {
    }

    Point3d(const Point3& p)
        : x(static_cast<double>(p.x_) * .001)
        , y(static_cast<double>(p.y_) * .001)
        , z(static_cast<double>(p.z_) * .001)
    {
    }

    Point3d operator+(const Point3d& p) const
    {
        return Point3d(x + p.x, y + p.y, z + p.z);
    }
    Point3d operator-(const Point3d& p) const
    {
        return Point3d(x - p.x, y - p.y, z - p.z);
    }
    Point3d operator*(const double f) const
    {
        return Point3d(x * f, y * f, z * f);
    }
    Point3d operator/(const double f) const
    {
        return Point3d(x / f, y / f, z / f);
    }

    Point3d& operator+=(const Point3d& p)
    {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }
    Point3d& operator-=(const Point3d& p)
    {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return *this;
    }
    Point3d& operator*=(const double f)
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }

    bool operator==(Point3d& p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(Point3d& p) const
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

    inline Point3d normalized() const
    {
        return (*this) / vSize();
    }

    Point3d cross(const Point3d& p) const
    {
        return Point3d(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }

    static Point3d cross(const Point3& a, const Point3& b)
    {
        return Point3d(a).cross(Point3d(b));
    }

    Point3 toPoint3()
    {
        return Point3(MM2INT(x), MM2INT(y), MM2INT(z));
    }
};

inline double operator*(Point3d lhs, const Point3d& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

} // namespace cura
#endif // POINT3D_H
