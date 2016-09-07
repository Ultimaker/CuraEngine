/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_LINEAR_ALG_2D_H
#define UTILS_LINEAR_ALG_2D_H

#include "intpoint.h"

namespace cura
{
class LinearAlg2D
{
public:
    static short pointLiesOnTheRightOfLine(Point p, Point p0, Point p1)
    {
        // no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
        if ( std::max(p0.X, p1.X) >= p.X )
        {
            int64_t pdY = p1.Y-p0.Y;
            if (pdY < 0) // p0->p1 is 'falling'
            {
                if ( p1.Y <= p.Y && p0.Y > p.Y ) // candidate
                {
                    // dx > 0 if intersection is to right of p.X
                    int64_t dx = (p1.X - p0.X) * (p1.Y - p.Y) - (p1.X-p.X)*pdY;
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
                    int64_t dx = (p1.X - p0.X) * (p.Y - p0.Y) - (p.X-p0.X)*pdY;
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
                        (pdY==0 && std::min(p0.X,p1.X) <= p.X) )
                    {
                        return 0;
                    }
                }
            }
        }
        return -1;
                
    }
    
    
    /*!
    * Find the point closest to \p from on the line from \p p0 to \p p1
    */
    static Point getClosestOnLineSegment(Point from, Point p0, Point p1)
    {
        Point direction = p1 - p0;
        Point toFrom = from-p0;
        int64_t projected_x = dot(toFrom, direction) ;

        int64_t x_p0 = 0;
        int64_t x_p1 = vSize2(direction);

        if (x_p1 == 0)
        {
    //         std::cout << "warning! too small segment" << std::endl;
            return p0;
        }
        if (projected_x <= x_p0)
        {
            return p0;
        }
        if (projected_x >= x_p1)
        {
            return p1;
        }
        else
        {
            Point ret = p0 + projected_x / vSize(direction) * direction  / vSize(direction);
            return ret ;
        }

    }



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
    static int64_t getDist2FromLineSegment(const Point& a, const Point& b, const Point& c, char* b_is_beyond_ac = nullptr)
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
        Point ac = c - a;
        int64_t ac_size = vSize(ac);

        Point ab = b - a;
        if (ac_size == 0) 
        {
            int64_t ab_dist2 = vSize2(ab); 
            if (ab_dist2 == 0)
            {
                *b_is_beyond_ac = 0; // a is on b is on c
            }
            // otherwise variable b_is_beyond_ac remains its value; it doesn't make sense to choose between -1 and 1
            return ab_dist2;
        }
        int64_t projected_x = dot(ab, ac);
        int64_t ax_size = projected_x / ac_size;
        
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
        Point ax = ac * ax_size / ac_size;
        Point bx = ab - ax;
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
    static bool lineSegmentsAreCloserThan(const Point& a, const Point& b, const Point& c, const Point& d, int64_t max_dist)
    {
        int64_t max_dist2 = max_dist * max_dist;

        return getDist2FromLineSegment(a, c, b) <= max_dist2
                || getDist2FromLineSegment(a, d, b) <= max_dist2
                || getDist2FromLineSegment(c, a, d) <= max_dist2
                || getDist2FromLineSegment(c, b, d) <= max_dist2;
    }
    
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
    static inline int64_t pointIsLeftOfLine(const Point& p, const Point& a, const Point& b)
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
    static bool getPointOnLineWithDist(const Point p, const Point a, const Point b, int64_t dist, Point& result);
};



}//namespace cura
#endif//UTILS_LINEAR_ALG_2D_H