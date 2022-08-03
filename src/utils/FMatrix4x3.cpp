//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/FMatrix4x3.h" //The definitions we're implementing.

#include "utils/floatpoint.h" //This matrix gets applied to floating point coordinates.
#include "utils/IntPoint.h" //Conversion directly into integer-based coordinates.
#include "settings/types/Ratio.h" //Scale factor.

namespace cura
{

FMatrix4x3 FMatrix4x3::scale(const Ratio scale, const Point3 origin)
{
    return FMatrix4x3::scale(scale, scale, scale, origin);
}

FMatrix4x3 FMatrix4x3::scale(const Ratio scale_x, const Ratio scale_y, const Ratio scale_z, const Point3 origin)
{
    FMatrix4x3 result;
    result.m[0][0] = scale_x; //X scale.
    result.m[1][1] = scale_y; //Y scale.
    result.m[2][2] = scale_z; //Z scale.

    //Apply a transformation to scale it away from the origin.
    result.m[3][0] = (scale_x - 1.0) * -origin.x; //Arrived at by manually applying an inverse move, the scale, then a move.
    result.m[3][1] = (scale_y - 1.0) * -origin.y;
    result.m[3][2] = (scale_z - 1.0) * -origin.z;

    return result;
}

FMatrix4x3::FMatrix4x3()
{
    m[0][0] = 1.0;
    m[1][0] = 0.0;
    m[2][0] = 0.0;
    m[3][0] = 0.0;
    m[0][1] = 0.0;
    m[1][1] = 1.0;
    m[2][1] = 0.0;
    m[3][1] = 0.0;
    m[0][2] = 0.0;
    m[1][2] = 0.0;
    m[2][2] = 1.0;
    m[3][2] = 0.0;
}

Point3 FMatrix4x3::apply(const FPoint3& p) const
{
    return Point3(
        MM2INT(p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0]),
        MM2INT(p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1]),
        MM2INT(p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + m[3][2])
    );
}

Point3 FMatrix4x3::apply(const Point3& p) const
{
    return Point3(
        m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z + m[3][0],
        m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z + m[3][1],
        m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z + m[3][2]
    );
}

}