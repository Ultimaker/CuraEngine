//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

namespace cura
{

Point3& operator Point3::*=(const Point3& p)
{
    x *= p.x;
    y *= p.y;
    z *= p.z;
}

}