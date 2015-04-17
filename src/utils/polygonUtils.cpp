/** Copyright (C) 2013 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "polygonUtils.h"

#include <list>

#include "../MACROS.h"
namespace cura {


    
    static bool convex3(int64_t x0, int64_t y0, int64_t x1, int64_t y1, int64_t x2, int64_t y2 )
    {
        return (y1-y0)*(x1-x2) > (x0-x1)*(y2-y1);
    }
    static bool convex3(Point const& p0, Point const &p1, Point const & p2 )
    {
        return convex3( p0.X, p0.Y, p1.X, p1.Y, p2.X, p2.Y );
    }

Polygon convexHull(PolygonRef poly)
{
    std::list<Point> result(poly.begin(), poly.end());
    
    DEBUG_SHOW(*poly.begin());
    DEBUG_SHOW(poly.back());
    
    auto getPrev = [&result](std::list<Point>::iterator i) 
    {
        if (i == result.begin())
            return --result.end();
        return --i;
    };
    auto getNext = [&result](std::list<Point>::iterator i) 
    {
        if (++i == result.end())
            return result.begin();
        return i;
    };
    
    for (std::list<Point>::iterator i = result.begin(); i != result.end(); i++)
    {
        Point& here = *i;
        Point& next = *getNext(i);
        Point& prev = *getPrev(i);
        
            DEBUG_SHOW(next);
            DEBUG_SHOW(here);
            DEBUG_SHOW(prev);
        
        while (!convex3(prev, here, next))
        {
            std::list<Point>::iterator here_it = result.erase(i)--;
            prev = *getPrev(here_it);
            here = *here_it;
            prev = *getNext(here_it);
            
            DEBUG_SHOW(next);
            DEBUG_SHOW(here);
            DEBUG_SHOW(prev);
        }
    }
    
    Polygon ret;
    for (Point& p : result)
        ret.add(p);
    return ret;
}
}//namespace cura
