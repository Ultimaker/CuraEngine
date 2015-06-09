/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_LINEAR_ALG_2D_H
#define UTILS_LINEAR_ALG_2D_H

#include "intpoint.h"

short pointLiesOnTheRightOfLine(Point p, Point p0, Point p1)
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






#endif//LINEAR_ALG_2D_H