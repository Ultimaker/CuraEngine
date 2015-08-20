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
    * \param a the first point of the line segment
    * \param b the point to measure the distance from
    * \param c the second point on the line segment
    * \param b_is_beyond_ac optional output parameter: whether \p b is closest to the line segment (0), to \p a (-1) or \p b (1)
    */
    static int64_t getDist2FromLineSegment(Point& a, Point& b, Point& c, char* b_is_beyond_ac)
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
        if (ac_size == 0) { return 0; }
        
        Point ab = b - a;
        int64_t projected_x = dot(ab, ac);
        int64_t ax_size = projected_x / ac_size;
        
        if (ax_size < 0) 
        {// b is 'before' segment ac 
            if (b_is_beyond_ac) { *b_is_beyond_ac = -1; }
            return vSize2(ab);
        }
        if (ax_size > ac_size)
        {// b is 'after' segment ac
            if (b_is_beyond_ac) { *b_is_beyond_ac = 1; }
            return vSize2(b - c);
        }
        
        Point ax = ac * ax_size / ac_size;
        Point bx = ab - ax;
        if (b_is_beyond_ac) { *b_is_beyond_ac = 0; }
        return vSize2(bx);
    }


};



}//namespace cura
#endif//UTILS_LINEAR_ALG_2D_H