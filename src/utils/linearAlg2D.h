//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_LINEAR_ALG_2D_H
#define UTILS_LINEAR_ALG_2D_H

#include "IntPoint.h"

namespace cura
{
class LinearAlg2D
{
public:
    static short pointLiesOnTheRightOfLine(const Point& p, const Point& p0, const Point& p1)
    {
        // no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
        if ( std::max(p0.X, p1.X) >= p.X )
        {
            const coord_t pd_y = p1.Y - p0.Y;
            if (pd_y < 0) // p0->p1 is 'falling'
            {
                if (p1.Y <= p.Y && p0.Y > p.Y) // candidate
                {
                    // dx > 0 if intersection is to right of p.X
                    const coord_t dx = (p1.X - p0.X) * (p1.Y - p.Y) - (p1.X - p.X) * pd_y;
                    if (dx == 0) // includes p == p1
                    {
                        return 0;
                    }
                    if (dx > 0)
                    {
                        return 1;
                    }
                }
            }
            else if (p.Y >= p0.Y)
            {
                if (p.Y < p1.Y) // candidate for p0->p1 'rising' and includes p.Y
                {
                    // dx > 0 if intersection is to right of p.X
                    const coord_t dx = (p1.X - p0.X) * (p.Y - p0.Y) - (p.X - p0.X) * pd_y;
                    if (dx == 0) // includes p == p0
                    {
                        return 0;
                    }
                    if (dx > 0)
                    {
                        return 1;
                    }
                }
                else if (p.Y == p1.Y)
                {
                    // some special cases here, points on border:
                    // - p1 exactly matches p (might otherwise be missed)
                    // - p0->p1 exactly horizontal, and includes p.
                    // (we already tested std::max(p0.X,p1.X) >= p.X )
                    if (p.X == p1.X ||
                        (pd_y == 0 && std::min(p0.X, p1.X) <= p.X) )
                    {
                        return 0;
                    }
                }
            }
        }
        return -1;
                
    }

    /*!
     * Find whether a point projected on a line segment would be projected to
     * - properly on the line : zero returned
     * - closer to \p a : -1 returned
     * - closer to \p b : 1 returned
     * 
     * \param from The point to check in relation to the line segment
     * \param a The start point of the line segment
     * \param b The end point of the line segment
     * \return the sign of the projection wrt the line segment
     */
    inline static short pointIsProjectedBeyondLine(const Point& from, const Point& a, const Point& b)
    {
        const Point vec = b - a;
        const Point point_vec = from - a;
        const coord_t dot_prod = dot(point_vec, vec);
        if (dot_prod < 0)
        { // point is projected to before ab
            return -1;
        }
        if (dot_prod > vSize2(vec))
        { // point is projected to after ab
            return 1;
        }
        return 0;
    }

    /*!
    * Find the point closest to \p from on the line from \p p0 to \p p1
    */
    static Point getClosestOnLineSegment(const Point& from, const Point& p0, const Point& p1)
    {
        const Point direction = p1 - p0;
        const Point to_from = from - p0;
        const coord_t projected_x = dot(to_from, direction) ;

        const coord_t x_p0 = 0;
        const coord_t x_p1 = vSize2(direction);

        if (x_p1 == 0)
        {
            //Line segment has length 0.
            return p0;
        }
        if (projected_x <= x_p0)
        {
            //Projection is beyond p0.
            return p0;
        }
        if (projected_x >= x_p1)
        {
            //Projection is beyond p1.
            return p1;
        }
        else
        {
            //Projection is between p0 and p1.
            //Return direction-normalised projection (projected_x / vSize(direction)) on direction vector.
            //vSize(direction) * vSize(direction) == vSize2(direction) == x_p1.
            return p0 + projected_x * direction / x_p1;
        }
    }

    /*!
     * Find the two points on two line segments closest to each other.
     * 
     * Find the smallest line segment connecting the two line segments a and b.
     * 
     * \param a1 first point on line a
     * \param a2 second point on line a
     * \param b1 first point on line b
     * \param b2 second point on line b
     * \return A pair: the first point on line a and the second pouint on line b
     */
    static std::pair<Point, Point> getClosestConnection(Point a1, Point a2, Point b1, Point b2);

    /*!
    * Get the squared distance from point \p b to a line *segment* from \p a to \p c.
    * 
    * In case \p b is on \p a or \p c, \p b_is_beyond_ac should become 0.
    * 
    * \param a the first point of the line segment
    * \param b the point to measure the distance from
    * \param c the second point on the line segment
    * \param b_is_beyond_ac optional output parameter: whether \p b is closest to the line segment (0), to \p a (-1) or \p b (1)
    */
    static coord_t getDist2FromLineSegment(const Point& a, const Point& b, const Point& c, int16_t* b_is_beyond_ac = nullptr)
    {
    /* 
    *     a,
    *     /|
    *    / |
    * b,/__|, x
    *   \  |
    *    \ |
    *     \|
    *      'c
    * 
    * x = b projected on ac
    * ax = ab dot ac / vSize(ac)
    * xb = ab - ax
    * error = vSize(xb)
    */
        const Point ac = c - a;
        const coord_t ac_size = vSize(ac);

        const Point ab = b - a;
        if (ac_size == 0) 
        {
            const coord_t ab_dist2 = vSize2(ab); 
            if (ab_dist2 == 0 && b_is_beyond_ac)
            {
                *b_is_beyond_ac = 0; // a is on b is on c
            }
            // otherwise variable b_is_beyond_ac remains its value; it doesn't make sense to choose between -1 and 1
            return ab_dist2;
        }
        const coord_t projected_x = dot(ab, ac);
        const coord_t ax_size = projected_x / ac_size;

        if (ax_size < 0) 
        {// b is 'before' segment ac 
            if (b_is_beyond_ac)
            {
                *b_is_beyond_ac = -1;
            }
            return vSize2(ab);
        }
        if (ax_size > ac_size)
        {// b is 'after' segment ac
            if (b_is_beyond_ac)
            {
                *b_is_beyond_ac = 1;
            }
            return vSize2(b - c);
        }

        if (b_is_beyond_ac)
        {
            *b_is_beyond_ac = 0;
        }
        const Point ax = ac * ax_size / ac_size;
        const Point bx = ab - ax;
        return vSize2(bx);
//         return vSize2(ab) - ax_size*ax_size; // less accurate
    }

    /*!
     * Checks whether the minimal distance between two line segments is at most \p max_dist
     * The first line semgent is given by end points \p a and \p b, the second by \p c and \p d.
     * 
     * \param a One end point of the first line segment
     * \param b Another end point of the first line segment
     * \param c One end point of the second line segment
     * \param d Another end point of the second line segment
     * \param max_dist The maximal distance between the two line segments for which this function will return true.
     */
    static bool lineSegmentsAreCloserThan(const Point& a, const Point& b, const Point& c, const Point& d, const coord_t max_dist)
    {
        const coord_t max_dist2 = max_dist * max_dist;

        return getDist2FromLineSegment(a, c, b) <= max_dist2
                || getDist2FromLineSegment(a, d, b) <= max_dist2
                || getDist2FromLineSegment(c, a, d) <= max_dist2
                || getDist2FromLineSegment(c, b, d) <= max_dist2;
    }

    /*!
     * Get the minimal distance between two line segments
     * The first line semgent is given by end points \p a and \p b, the second by \p c and \p d.
     * 
     * \param a One end point of the first line segment
     * \param b Another end point of the first line segment
     * \param c One end point of the second line segment
     * \param d Another end point of the second line segment
     */
    static coord_t getDist2BetweenLineSegments(const Point& a, const Point& b, const Point& c, const Point& d)
    {
        return
            std::min(getDist2FromLineSegment(a, c, b), 
            std::min(getDist2FromLineSegment(a, d, b),
            std::min(getDist2FromLineSegment(c, a, d),
                     getDist2FromLineSegment(c, b, d))));
    }

    /*!
     * Check whether two line segments collide.
     * 
     * \warning Edge cases (end points of line segments fall on other line segment) register as a collision.
     * 
     * \note All points are assumed to be transformed by the transformation matrix of the vector from \p a_from to \p a_to.
     * I.e. a is a vertical line; the Y of \p a_from_transformed is the same as the Y of \p a_to_transformed.
     * 
     * \param a_from_transformed The transformed from location of line a
     * \param a_from_transformed The transformed to location of line a
     * \param b_from_transformed The transformed from location of line b
     * \param b_from_transformed The transformed to location of line b
     * \return Whether the two line segments collide
     */
    static bool lineSegmentsCollide(const Point& a_from_transformed, const Point& a_to_transformed, Point b_from_transformed, Point b_to_transformed);

    /*!
     * Compute the angle between two consecutive line segments.
     * 
     * The angle is computed from the left side of b when looking from a.
     * 
     *   c
     *    \                     .
     *     \ b
     * angle|
     *      |
     *      a
     * 
     * \param a start of first line segment
     * \param b end of first segment and start of second line segment
     * \param c end of second line segment
     * \return the angle in radians between 0 and 2 * pi of the corner in \p b
     */
    static float getAngleLeft(const Point& a, const Point& b, const Point& c);

    /*!
     * Returns the determinant of the 2D matrix defined by the the vectors ab and ap as rows.
     * 
     * The returned value is zero for \p p lying (approximately) on the line going through \p a and \p b
     * The value is positive for values lying to the left and negative for values lying to the right when looking from \p a to \p b.
     * 
     * \param p the point to check
     * \param a the from point of the line
     * \param b the to point of the line
     * \return a positive value when \p p lies to the left of the line from \p a to \p b
     */
    static inline coord_t pointIsLeftOfLine(const Point& p, const Point& a, const Point& b)
    {
        return (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);
    }

    /*!
     * Get a point on the line segment (\p a - \p b)with a given distance to point \p p
     * 
     * In case there are two possible point that meet the criteria, choose the one closest to a.
     * 
     * \param p The reference point
     * \param a Start of the line segment
     * \param b End of the line segment
     * \param dist The required distance of \p result to \p p
     * \param[out] result The result (if any was found)
     * \return Whether any such point has been found
     */
    static bool getPointOnLineWithDist(const Point& p, const Point& a, const Point& b, const coord_t dist, Point& result);

    /*!
     * Get the squared distance from a point \p p to the line on which \p a and
     * \p b lie
     * \param p The point to measure the distance from.
     * \param a One of the points through which the line goes.
     * \param b One of the points through which the line goes.
     * \return The distance between the point and the line, squared.
     */
    static coord_t getDist2FromLine(const Point& p, const Point& a, const Point& b);

    /*!
     * Check whether a corner is acute or obtuse.
     * 
     * This function is irrespective of the order between \p a and \p c;
     * the lowest angle among bot hsides of the corner is always chosen.
     * 
     * isAcuteCorner(a, b, c) === isAcuteCorner(c, b, a)
     * 
     * \param a start of first line segment
     * \param b end of first segment and start of second line segment
     * \param c end of second line segment
     * \return positive if acute, negative if obtuse, zero if 90 degree corner
     */
    static inline int isAcuteCorner(const Point& a, const Point& b, const Point& c)
    {
        const Point ba = a - b;
        const Point bc = c - b;
        return dot(ba, bc);
    }

    /*!
     * Get the rotation matrix for rotating around a specific point in place.
     */
    static Point3Matrix rotateAround(const Point& middle, double rotation)
    {
        PointMatrix rotation_matrix(rotation);
        Point3Matrix rotation_matrix_homogeneous(rotation_matrix);
        return Point3Matrix::translate(middle).compose(rotation_matrix_homogeneous).compose(Point3Matrix::translate(-middle));
    }
};



}//namespace cura
#endif//UTILS_LINEAR_ALG_2D_H
