// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/linearAlg2D.h"

#include <algorithm> // swap
#include <cassert>
#include <cmath> // atan2
#include <numbers>

#include "geometry/Point3Matrix.h"
#include "geometry/PointMatrix.h"
#include "utils/math.h"

namespace cura
{

double LinearAlg2D::getAngleLeft(const Point2LL& a, const Point2LL& b, const Point2LL& c)
{
    const Point2LL ba = a - b;
    const Point2LL bc = c - b;
    const coord_t dott = dot(ba, bc); // dot product
    const coord_t det = ba.X * bc.Y - ba.Y * bc.X; // determinant
    if (det == 0)
    {
        if ((ba.X != 0 && (ba.X > 0) == (bc.X > 0)) || (ba.X == 0 && (ba.Y > 0) == (bc.Y > 0)))
        {
            return 0; // pointy bit
        }
        else
        {
            return std::numbers::pi; // straight bit
        }
    }
    const double angle = -atan2(det, dott); // from -pi to pi
    if (angle >= 0)
    {
        return angle;
    }
    else
    {
        return std::numbers::pi * 2 + angle;
    }
}


bool LinearAlg2D::getPointOnLineWithDist(const Point2LL& p, const Point2LL& a, const Point2LL& b, const coord_t dist, Point2LL& result)
{
    //         result
    //         v
    //   b<----r---a.......x
    //          '-.        :
    //              '-.    :
    //                  '-.p
    const Point2LL ab = b - a;
    const coord_t ab_size = vSize(ab);
    const Point2LL ap = p - a;
    const coord_t ax_size = (ab_size < 50) ? dot(normal(ab, 1000), ap) / 1000 : dot(ab, ap) / ab_size;
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


std::pair<Point2LL, Point2LL> LinearAlg2D::getClosestConnection(Point2LL a1, Point2LL a2, Point2LL b1, Point2LL b2)
{
    Point2LL b1_on_a = getClosestOnLineSegment(b1, a1, a2);
    coord_t b1_on_a_dist2 = vSize2(b1_on_a - b1);
    Point2LL b2_on_a = getClosestOnLineSegment(b2, a1, a2);
    coord_t b2_on_a_dist2 = vSize2(b2_on_a - b2);
    Point2LL a1_on_b = getClosestOnLineSegment(a1, b1, b2);
    coord_t a1_on_b_dist2 = vSize2(a1_on_b - a1);
    Point2LL a2_on_b = getClosestOnLineSegment(a1, b1, b2);
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

bool LinearAlg2D::lineSegmentsCollide(const Point2LL& a_from_transformed, const Point2LL& a_to_transformed, Point2LL b_from_transformed, Point2LL b_to_transformed)
{
    assert(std::abs(a_from_transformed.Y - a_to_transformed.Y) < 2 && "line a is supposed to be transformed to be aligned with the X axis!");
    assert(a_from_transformed.X - 2 <= a_to_transformed.X && "line a is supposed to be aligned with X axis in positive direction!");
    if ((b_from_transformed.Y >= a_from_transformed.Y && b_to_transformed.Y <= a_from_transformed.Y)
        || (b_to_transformed.Y >= a_from_transformed.Y && b_from_transformed.Y <= a_from_transformed.Y))
    {
        if (b_to_transformed.Y == b_from_transformed.Y)
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
            const coord_t x
                = b_from_transformed.X + (b_to_transformed.X - b_from_transformed.X) * (a_from_transformed.Y - b_from_transformed.Y) / (b_to_transformed.Y - b_from_transformed.Y);
            if (x >= a_from_transformed.X && x <= a_to_transformed.X)
            {
                return true;
            }
        }
    }
    return false;
}

coord_t LinearAlg2D::getDist2FromLine(const Point2LL& p, const Point2LL& a, const Point2LL& b)
{
    // NOTE: The version that tried to do a faster calulation wasn't actually that much faster, and introduced errors.
    //       Use this for now, should we need this, we can reimplement later.
    const auto dist = getDistFromLine(p, a, b);
    return dist * dist;
}

bool LinearAlg2D::isInsideCorner(const Point2LL a, const Point2LL b, const Point2LL c, const Point2LL query_point)
{
    /*
     Visualisation for the algorithm below:

                 query
                   |
                   |
                   |
    perp-----------b
                  / \       (note that the lines
                 /   \      AB and AC are normalized
                /     \     to 10000 units length)
               a       c
     */


    constexpr coord_t normal_length = 10000; // Create a normal vector of reasonable length in order to reduce rounding error.
    const Point2LL ba = normal(a - b, normal_length);
    const Point2LL bc = normal(c - b, normal_length);
    const Point2LL bq = query_point - b;
    const Point2LL perpendicular = turn90CCW(bq); // The query projects to this perpendicular to coordinate 0.
    const coord_t project_a_perpendicular = dot(ba, perpendicular); // Project vertex A on the perpendicular line.
    const coord_t project_c_perpendicular = dot(bc, perpendicular); // Project vertex C on the perpendicular line.
    if ((project_a_perpendicular > 0) != (project_c_perpendicular > 0)) // Query is between A and C on the projection.
    {
        return project_a_perpendicular > 0; // Due to the winding order of corner ABC, this means that the query is inside.
    }
    else // Beyond either A or C, but it could still be inside of the polygon.
    {
        const coord_t project_a_parallel = dot(ba, bq); // Project not on the perpendicular, but on the original.
        const coord_t project_c_parallel = dot(bc, bq);

        // Either:
        //  * A is to the right of B (project_a_perpendicular > 0) and C is below A (project_c_parallel < project_a_parallel), or
        //  * A is to the left of B (project_a_perpendicular < 0) and C is above A (project_c_parallel > project_a_parallel).
        return (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0);
    }
}

coord_t LinearAlg2D::getDistFromLine(const Point2LL& p, const Point2LL& a, const Point2LL& b)
{
    //  x.......a------------b
    //  :
    //  :
    //  p
    // return px_size
    const Point2LL vab = b - a;
    const Point2LL vap = p - a;
    const double ab_size = vSize(vab);
    if (ab_size == 0) // Line of 0 length. Assume it's a line perpendicular to the direction to p.
    {
        return vSize(vap);
    }
    const coord_t area_times_two = std::abs((p.X - b.X) * (p.Y - a.Y) + (a.X - p.X) * (p.Y - b.Y)); // Shoelace formula, factored
    const coord_t px_size = area_times_two / ab_size;
    return px_size;
}

Point2LL LinearAlg2D::getBisectorVector(const Point2LL& intersect, const Point2LL& a, const Point2LL& b, const coord_t vec_len)
{
    const auto a0 = a - intersect;
    const auto b0 = b - intersect;
    return (((a0 * vec_len) / std::max(1LL, vSize(a0))) + ((b0 * vec_len) / std::max(1LL, vSize(b0)))) / 2;
}

Point3Matrix LinearAlg2D::rotateAround(const Point2LL& middle, double rotation)
{
    PointMatrix rotation_matrix(rotation);
    Point3Matrix rotation_matrix_homogeneous(rotation_matrix);
    return Point3Matrix::translate(middle).compose(rotation_matrix_homogeneous).compose(Point3Matrix::translate(-middle));
}

bool LinearAlg2D::lineLineIntersection(const Point2LL& p1, const Point2LL& p2, const Point2LL& p3, const Point2LL& p4, float& t, float& u)
{
    const float x1mx2 = p1.X - p2.X;
    const float x1mx3 = p1.X - p3.X;
    const float x3mx4 = p3.X - p4.X;
    const float y1my2 = p1.Y - p2.Y;
    const float y1my3 = p1.Y - p3.Y;
    const float y3my4 = p3.Y - p4.Y;

    t = x1mx3 * y3my4 - y1my3 * x3mx4;
    u = x1mx3 * y1my2 - y1my3 * x1mx2;
    const float div = x1mx2 * y3my4 - y1my2 * x3mx4;
    if (div == 0.0f)
    {
        return false;
    }

    // NOTE: In theory the comparison 0 <= par <= 1 can now done without division for each parameter (as an early-out),
    //       but this is easier & when the intersection _does_ happen and we want the normalized parameters returned anyway.
    t /= div;
    u /= div;
    return true;
}

bool LinearAlg2D::segmentSegmentIntersection(const Point2LL& p1, const Point2LL& p2, const Point2LL& p3, const Point2LL& p4, float& t, float& u)
{
    return lineLineIntersection(p1, p2, p3, p4, t, u) && t >= 0.0f && u >= 0.0f && t <= 1.0f && u <= 1.0f;
}

bool LinearAlg2D::lineLineIntersection(const Point2LL& a, const Point2LL& b, const Point2LL& c, const Point2LL& d, Point2LL& output)
{
    float t, u;
    if (! lineLineIntersection(a, b, c, d, t, u))
    {
        return false;
    }
    const Point2LL result = a + (b - a) * t;
    if (std::abs(result.X) > std::numeric_limits<int32_t>::max() || std::abs(result.Y) > std::numeric_limits<int32_t>::max())
    {
        // Intersection is so far away that it could lead to integer overflows.
        // Even though the lines aren't 100% parallel, it's better to pretend they are. They are practically parallel.
        return false;
    }
    output = result;
    return true;
}

} // namespace cura
