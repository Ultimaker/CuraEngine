//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Point3.h" //The headers we're implementing.

namespace cura
{

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

}