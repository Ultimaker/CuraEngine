//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "linearAlg2D.h"

#include <cmath> // atan2
#include <cassert>
#include <algorithm> // swap

#include "IntPoint.h" // dot

namespace cura 
{

float LinearAlg2D::getAngleLeft(const Point& a, const Point& b, const Point& c)
{
    const Point ba = a - b;
    const Point bc = c - b;
    const coord_t dott = dot(ba, bc); // dot product
    const coord_t det = ba.X * bc.Y - ba.Y * bc.X; // determinant
    if (det == 0)
    {
        if (
            (ba.X != 0 && (ba.X > 0) == (bc.X > 0))
            || (ba.X == 0 && (ba.Y > 0) == (bc.Y > 0))
            )
        {
            return 0; // pointy bit
        }
        else
        {
            return M_PI; // straight bit
        }
    }
    const float angle = -atan2(det, dott); // from -pi to pi
    if (angle >= 0)
    {
        return angle;
    }
    else 
    {
        return M_PI * 2 + angle;
    }
}


bool LinearAlg2D::getPointOnLineWithDist(const Point& p, const Point& a, const Point& b, const coord_t dist, Point& result)
{
    //         result
    //         v
    //   b<----r---a.......x
    //          '-.        :
    //              '-.    :
    //                  '-.p
    const Point ab = b - a;
    const coord_t ab_size = vSize(ab);
    const Point ap = p - a;
    const coord_t ax_size = (ab_size < 50)? dot(normal(ab, 1000), ap) / 1000 : dot(ab, ap) / ab_size;
    const coord_t ap_size2 = vSize2(ap);
    const coord_t px_size = sqrt(std::max(coord_t(0), ap_size2 - ax_size * ax_size));
    if (px_size > dist)
    {
        return false;
    }
    const coord_t xr_size = sqrt(dist * dist - px_size * px_size);
    if (ax_size <= 0)
    { // x lies before ab
        const coord_t ar_size = xr_size + ax_size;
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
        const coord_t ar_size = ax_size - xr_size;
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
        const coord_t ar1_size = ax_size - xr_size;
        if (ar1_size >= 0)
        {
            result = a + normal(ab, ar1_size);
            return true;
        }
        const coord_t ar2_size = ax_size + xr_size;
        if (ar2_size < ab_size)
        {
            result = a + normal(ab, ar2_size);
            return true;
        }
        return false;
    }
}


std::pair<Point, Point> LinearAlg2D::getClosestConnection(Point a1, Point a2, Point b1, Point b2)
{
    Point b1_on_a = getClosestOnLineSegment(b1, a1, a2);
    coord_t b1_on_a_dist2 = vSize2(b1_on_a - b1);
    Point b2_on_a = getClosestOnLineSegment(b2, a1, a2);
    coord_t b2_on_a_dist2 = vSize2(b2_on_a - b2);
    Point a1_on_b = getClosestOnLineSegment(a1, b1, b2);
    coord_t a1_on_b_dist2 = vSize2(a1_on_b - a1);
    Point a2_on_b = getClosestOnLineSegment(a1, b1, b2);
    coord_t a2_on_b_dist2 = vSize2(a2_on_b - a2);
    if (b1_on_a_dist2 < b2_on_a_dist2 && b1_on_a_dist2 < a1_on_b_dist2 && b1_on_a_dist2 < a2_on_b_dist2)
    {
        return std::make_pair(b1_on_a, b1);
    }
    else if (b2_on_a_dist2 < a1_on_b_dist2 && b2_on_a_dist2 < a2_on_b_dist2)
    {
        return std::make_pair(b2_on_a, b2);
    }
    else if (a1_on_b_dist2 < a2_on_b_dist2)
    {
        return std::make_pair(a1, a1_on_b);
    }
    else
    {
        return std::make_pair(a2, a2_on_b);
    }
}

bool LinearAlg2D::lineSegmentsCollide(const Point& a_from_transformed, const Point& a_to_transformed, Point b_from_transformed, Point b_to_transformed)
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
            const coord_t x = b_from_transformed.X + (b_to_transformed.X - b_from_transformed.X) * (a_from_transformed.Y - b_from_transformed.Y) / (b_to_transformed.Y - b_from_transformed.Y);
            if (x >= a_from_transformed.X && x <= a_to_transformed.X)
            {
                return true;
            }
        }
    }
    return false;
}

coord_t LinearAlg2D::getDist2FromLine(const Point& p, const Point& a, const Point& b)
{
    constexpr coord_t SQRT_LLONG_MAX_FLOOR = 3037000499;

    //  x.......a------------b
    //  :
    //  :
    //  p
    // return px_size^2 (if there is no overflow)
    const Point vab = b - a;
    const Point vap = p - a;
    const coord_t ab_size2 = vSize2(vab);
    const coord_t ap_size2 = vSize2(vap);
    coord_t px_size2;
    if(ab_size2 == 0) //Line of 0 length. Assume it's a line perpendicular to the direction to p.
    {
        return ap_size2;
    }
    const coord_t dott = dot(vab, vap);
    if (dott != 0 && abs(dott) > SQRT_LLONG_MAX_FLOOR)
    { // dott * dott will overflow so calculate px_size2 via its square root
        coord_t px_size = LinearAlg2D::getDistFromLine(p, a, b);
        if (px_size <= SQRT_LLONG_MAX_FLOOR)
        {
            // Due to rounding and conversion errors, this multiplication may not be the exact value that would be
            // produced via the dott product, but it should still be close enough
            px_size2 = px_size * px_size;
        }
        else
        {
            px_size2 = std::numeric_limits<long long>::max();
        }
    }
    else
    {
        const coord_t ax_size2 = dott * dott / ab_size2;
        px_size2 = std::max(coord_t(0), ap_size2 - ax_size2);
    }
    return px_size2;
}

coord_t LinearAlg2D::getDistFromLine(const Point& p, const Point& a, const Point& b)
{
    //  x.......a------------b
    //  :
    //  :
    //  p
    // return px_size
    const Point vab = b - a;
    const Point vap = p - a;
    const double ab_size = vSize(vab);
    if(ab_size == 0) //Line of 0 length. Assume it's a line perpendicular to the direction to p.
    {
        return vSize(vap);
    }
    const coord_t area_times_two = abs((p.X - b.X) * (p.Y - a.Y) + (a.X - p.X) * (p.Y - b.Y)); // Shoelace formula, factored
    const coord_t px_size = area_times_two / ab_size;
    return px_size;
}

} // namespace cura
