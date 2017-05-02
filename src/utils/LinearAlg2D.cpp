/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "linearAlg2D.h"

#include <cmath> // atan2
#include <cassert>
#include <algorithm> // swap

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
    const int64_t ax_size = (ab_size < 50)? dot(normal(ab, 1000), ap) / 1000 : dot(ab, ap) / ab_size;
    const int64_t ap_size2 = vSize2(ap);
    const int64_t px_size = sqrt(std::max(int64_t(0), ap_size2 - ax_size * ax_size));
    if (px_size > dist)
    {
        return false;
    }
    const int64_t xr_size = sqrt(dist * dist - px_size * px_size);
    if (ax_size <= 0)
    { // x lies before ab
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

bool LinearAlg2D::lineSegmentsCollide(Point a_from_transformed, Point a_to_transformed, Point b_from_transformed, Point b_to_transformed)
{
    assert(std::abs(a_from_transformed.Y - a_to_transformed.Y) < 2 && "line a is supposed to be transformed to be aligned with the X axis!");
    assert(a_from_transformed.X - 2 <= a_to_transformed.X && "line a is supposed to be aligned with X axis in positive direction!");
    if ((b_from_transformed.Y >= a_from_transformed.Y && b_to_transformed.Y <= a_from_transformed.Y) || (b_to_transformed.Y >= a_from_transformed.Y && b_from_transformed.Y <= a_from_transformed.Y))
    {
        if(b_to_transformed.Y == b_from_transformed.Y)
        {
            if (b_to_transformed.X < b_from_transformed.X)
            {
                std::swap(b_to_transformed.X, b_from_transformed.X);
            }
            if (b_from_transformed.X > a_to_transformed.X)
            {
                return false;
            }
            if (b_to_transformed.X < a_from_transformed.X)
            {
                return false;
            }
            return true;
        }
        else
        {
            int64_t x = b_from_transformed.X + (b_to_transformed.X - b_from_transformed.X) * (a_from_transformed.Y - b_from_transformed.Y) / (b_to_transformed.Y - b_from_transformed.Y);
            if (x >= a_from_transformed.X && x <= a_to_transformed.X)
            {
                return true;
            }
        }
    }
    return false;
}

} // namespace cura
