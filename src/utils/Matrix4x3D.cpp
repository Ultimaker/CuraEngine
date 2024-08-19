// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Matrix4x3D.h" //The definitions we're implementing.

#include "geometry/Point2LL.h" //Conversion directly into integer-based coordinates.
#include "settings/types/Ratio.h" //Scale factor.
#include "utils/Point3D.h" //This matrix gets applied to floating point coordinates.

namespace cura
{

Matrix4x3D Matrix4x3D::scale(const Ratio scale, const Point3LL origin)
{
    return Matrix4x3D::scale(scale, scale, scale, origin);
}

Matrix4x3D Matrix4x3D::scale(const Ratio scale_x, const Ratio scale_y, const Ratio scale_z, const Point3LL origin)
{
    Matrix4x3D result;
    result.m[0][0] = scale_x; // X scale.
    result.m[1][1] = scale_y; // Y scale.
    result.m[2][2] = scale_z; // Z scale.

    // Apply a transformation to scale it away from the origin.
    result.m[3][0] = (scale_x - 1.0) * -origin.x_; // Arrived at by manually applying an inverse move, the scale, then a move.
    result.m[3][1] = (scale_y - 1.0) * -origin.y_;
    result.m[3][2] = (scale_z - 1.0) * -origin.z_;

    return result;
}

Matrix4x3D::Matrix4x3D()
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

Point3LL Matrix4x3D::apply(const Point3D& p) const
{
    return Point3LL(
        MM2INT(p.x_ * m[0][0] + p.y_ * m[1][0] + p.z_ * m[2][0] + m[3][0]),
        MM2INT(p.x_ * m[0][1] + p.y_ * m[1][1] + p.z_ * m[2][1] + m[3][1]),
        MM2INT(p.x_ * m[0][2] + p.y_ * m[1][2] + p.z_ * m[2][2] + m[3][2]));
}

Point3LL Matrix4x3D::apply(const Point3LL& p) const
{
    return Point3LL(
        m[0][0] * p.x_ + m[1][0] * p.y_ + m[2][0] * p.z_ + m[3][0],
        m[0][1] * p.x_ + m[1][1] * p.y_ + m[2][1] * p.z_ + m[3][1],
        m[0][2] * p.x_ + m[1][2] * p.y_ + m[2][2] * p.z_ + m[3][2]);
}

} // namespace cura
