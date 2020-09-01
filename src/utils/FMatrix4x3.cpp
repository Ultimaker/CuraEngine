//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FMatrix4x3.h" //The definitions we're implementing.

#include "floatpoint.h" //This matrix gets applied to floating point coordinates.
#include "IntPoint.h" //Conversion directly into integer-based coordinates.
#include "../settings/types/Ratio.h" //Scale factor.

namespace cura
{
    
FMatrix4x3 FMatrix4x3::scale(const Ratio scale)
{
    FMatrix4x3 result;
    result.m[0][0] = scale; //X.
    result.m[1][1] = scale; //Y.
    result.m[2][2] = scale; //Z.
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
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
}

Point3 FMatrix4x3::apply(const FPoint3& p) const
{
    return Point3(
        MM2INT(p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0]),
        MM2INT(p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1]),
        MM2INT(p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + m[3][2]));
}

}