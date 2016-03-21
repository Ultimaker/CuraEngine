/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "linearAlg2D.h"

#include <cmath> // atan2

#include "intpoint.h" // dot

namespace cura 
{

float LinearAlg2D::getAngleLeft(const Point& a, const Point& b, const Point& c)
{
    Point ba = a - b;
    Point bc = c - b;
    int64_t dott = dot(ba, bc); // dot product
    int64_t det = ba.X * bc.Y - ba.Y * bc.X; // determinant
    float angle = -atan2(det, dott); // from -pi to pi
    if (angle >= 0 )
    {
        return angle;
    }
    else 
    {
        return M_PI * 2 + angle;
    }
}


} // namespace cura 
