//Copyright (c) 2018 Ultimaker B.V.
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
bool LinearAlg2D::areParallel(LineSegment a, LineSegment b, coord_t allowed_error)
{
    Point a_vec = a.getVector();
    Point b_vec = b.getVector();
    coord_t a_size = vSize(a_vec);
    coord_t b_size = vSize(b_vec);
    if (a_size == 0 || b_size == 0)
    {
        return true;
    }
    coord_t dot_size = std::abs(dot(a_vec, b_vec));
    coord_t dot_diff = std::abs(dot_size - a_size * b_size);
    coord_t allowed_dot_error = allowed_error * std::sqrt(dot_size);
    return dot_diff < allowed_dot_error;
}

bool LinearAlg2D::areCollinear(LineSegment a, LineSegment b, coord_t allowed_error)
{
    bool lines_are_parallel = areParallel(a, b, allowed_error);
    bool to_b_from_is_on_line = areParallel(LineSegment(a.from, b.from), b, allowed_error);
    bool to_b_to_is_on_line = areParallel(LineSegment(a.from, b.to), b, allowed_error);
    return lines_are_parallel && to_b_from_is_on_line && to_b_to_is_on_line;
}

coord_t LinearAlg2D::projectedLength(LineSegment to_project, LineSegment onto)
{
    const Point& a = onto.from;
    const Point& b = onto.to;
    const Point& c = to_project.from;
    const Point& d = to_project.to;
    Point cd = d - c;
    coord_t cd_size = vSize(cd);
    assert(cd_size > 0);
    Point ca = a - c;
    coord_t a_projected = dot(ca, cd) / cd_size;
    Point cb = b - c;
    coord_t b_projected = dot(cb, cd) / cd_size;
    return b_projected - a_projected;
}

LineSegment LinearAlg2D::project(LineSegment to_project, LineSegment onto)
{
    return LineSegment(project(to_project.from, onto), project(to_project.to, onto));
}

Point LinearAlg2D::project(Point p, LineSegment onto)
{
    const Point& a = onto.from;
    const Point& b = onto.to;
    Point ab = b - a;
    coord_t ab_size = vSize(ab);
    assert(ab_size > 0);
    Point pa = p - onto.from;
    coord_t projected_length = dot(ab, pa) / ab_size;
    return onto.from + normal(ab, projected_length);
}

coord_t LinearAlg2D::getTriangleArea(Point a, Point b)
{
    return std::abs(a.X * b.Y - a.Y * b.X) / 2;
}

coord_t LinearAlg2D::getDist2FromLine(const Point& p, const Point& a, const Point& b)
{
    //  x.......a------------b
    //  :
    //  :
    //  p
    // return px_size
    assert(a != b);  // the line can't be a point
    const Point vab = b - a;
    const Point vap = p - a;
    const coord_t dott = dot(vab, vap);
    const coord_t ax_size2 = dott * dott / vSize2(vab);
    const coord_t ap_size2 = vSize2(vap);
    const coord_t px_size2 = std::max(coord_t(0), ap_size2 - ax_size2);
    return px_size2;
}

} // namespace cura
