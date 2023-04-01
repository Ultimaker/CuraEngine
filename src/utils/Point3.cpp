//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Point3.h" //The headers we're implementing.

namespace cura
{

Point3 Point3::operator +(const Point3& p) const
{
    return Point3(x + p.x, y + p.y, z + p.z);
}

Point3 Point3::operator -() const
{
    return Point3(-x, -y, -z);
}

Point3 Point3::operator -(const Point3& p) const
{
    return Point3(x - p.x, y - p.y, z - p.z);
}

Point3 Point3::operator *(const Point3& p) const
{
    return Point3(x * p.x, y * p.y, z * p.z);
}

Point3 Point3::operator /(const Point3& p) const
{
    return Point3(x / p.x, y / p.y, z / p.z);
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

Point3& Point3::operator /=(const Point3& p)
{
    x /= p.x;
    y /= p.y;
    z /= p.z;
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

}
