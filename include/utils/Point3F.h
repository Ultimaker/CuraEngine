// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT3F_H
#define POINT3F_H

#include <math.h>
#include <stdint.h>

#include "Point3D.h"
#include "geometry/Point2LL.h"


namespace cura
{

/*
Floating point 3D points are used during model loading as 3D vectors.
They represent millimeters in 3D space.
This class should not be used for geometric computation. Use Point3d for this purpose.
*/
class Point3F
{
public:
    float x_, y_, z_;

    Point3F()
    {
    }

    Point3F(float x, float y, float z)
        : x_(x)
        , y_(y)
        , z_(z)
    {
    }

    Point3D toPoint3d() const
    {
        return Point3D(static_cast<double>(x_), static_cast<double>(y_), static_cast<double>(z_));
    }
};

} // namespace cura
#endif // POINT3F_H
