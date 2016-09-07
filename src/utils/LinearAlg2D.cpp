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


bool LinearAlg2D::getPointOnLineWithDist(const Point p, const Point a, const Point b, int64_t dist, Point& result)
{
    //         result
    //         v
    //   b<----r---a.......x
    //          '-.        :
    //              '-.    :
    //                  '-.p
    const Point ab = b - a;
    const int64_t ab_size = vSize(ab);
    const Point ap = p - a;
    const int64_t ap_size = vSize(ap);
    const int64_t dott = dot(ab, ap);
    const int64_t ax_size = dott / ab_size;
    const int64_t px_size = sqrt(ap_size * ap_size - ax_size * ax_size);
    if (px_size > dist)
    {
        return false;
    }
    const int64_t xr_size = sqrt(dist * dist - px_size * px_size);
    if (ax_size <= 0)
    { // x lies before ba
        const int64_t ar_size = xr_size + ax_size;
        if (ar_size < 0 || ar_size > ab_size)
        { // r lies outisde of ab
            return false;
        }
        else
        {
            result = a + normal(ab, ar_size);
            return true;
        }
    }
    else if (ax_size >= ab_size)
    { // x lies after ab
        //         result
        //         v
        //   a-----r-->b.......x
        //          '-.        :
        //              '-.    :
        //                  '-.p
        const int64_t ar_size = ax_size - xr_size;
        if (ar_size < 0 || ar_size > ab_size)
        { // r lies outisde of ab
            return false;
        }
        else
        {
            result = a + normal(ab, ar_size);
            return true;
        }
    }
    else // ax_size > 0 && ax_size < ab_size
    { // x lies on ab
        //            result is either or
        //         v                       v
        //   a-----r-----------x-----------r----->b
        //          '-.        :        .-'
        //              '-.    :    .-'
        //                  '-.p.-'
        //           or there is not result:
        //         v                       v
        //         r   a-------x---->b     r
        //          '-.        :        .-'
        //              '-.    :    .-'
        //                  '-.p.-'
        // try r in both directions
        const int64_t ar1_size = ax_size - xr_size;
        if (ar1_size >= 0)
        {
            result = a + normal(ab, ar1_size);
            return true;
        }
        const int64_t ar2_size = ax_size + xr_size;
        if (ar2_size < ab_size)
        {
            result = a + normal(ab, ar2_size);
            return true;
        }
        return false;
    }
}

} // namespace cura