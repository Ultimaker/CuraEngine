//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Point3.h" //The headers we're implementing.

namespace cura
{

Point3 Point3::operator +(const Point3& p) const
{
    return Point3(x + p.x, y + p.y, z + p.z);
}

Point3 Point3::operator -(const Point3& p) const
{
    return Point3(x - p.x, y - p.y, z - p.z);
}

Point3 Point3::operator *(const Point3& p) const
{
    return Point3(x * p.x, y * p.y, z * p.z);
}

Point3 Point3::operator *(const int32_t i) const
{
    return Point3(x * i, y * i, z * i);
}

Point3 Point3::operator *(const double d) const
{
    return Point3(d * x, d * y, d * z);
}

Point3 Point3::operator /(const Point3& p) const
{
    return Point3(x / p.x, y / p.y, z / p.z);
}

Point3 Point3::operator /(const int32_t i) const
{
    return Point3(x / i, y / i, z / i);
}

Point3 Point3::operator /(const double d) const
{
    return Point3(x / d, y / d, z / d);
}

Point3& Point3::operator +=(const Point3& p)
{
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
}

Point3& Point3::operator -=(const Point3& p)
{
    x -= p.x;
    y -= p.y;
    z -= p.z;
    return *this;
}

Point3& Point3::operator *=(const Point3& p)
{
    x *= p.x;
    y *= p.y;
    z *= p.z;
    return *this;
}

Point3& Point3::operator *=(const int32_t i)
{
    x *= i;
    y *= i;
    z *= i;
    return *this;
}

Point3& Point3::operator *=(const double d)
{
    x *= d;
    y *= d;
    z *= d;
    return *this;
}

Point3& Point3::operator /=(const Point3& p)
{
    x /= p.x;
    y /= p.y;
    z /= p.z;
    return *this;
}

Point3& Point3::operator /=(const int32_t i)
{
    x /= i;
    y /= i;
    z /= i;
    return *this;
}

Point3& Point3::operator /=(const double d)
{
    x /= d;
    y /= d;
    z /= d;
    return *this;
}

bool Point3::operator ==(const Point3& p) const
{
    return x == p.x && y == p.y && z == p.z;
}

bool Point3::operator !=(const Point3& p) const
{
    return x != p.x || y != p.y || z != p.z;
}

inline Point3 operator *(const int32_t i, const Point3& rhs)
{
    return rhs * i;
}

inline Point3 operator *(const double d, const Point3& rhs)
{
    return rhs * d;
}

}