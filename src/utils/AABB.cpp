/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "AABB.h"

#include <limits>

namespace cura
{


AABB::AABB()
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
}

AABB::AABB(Point&min, Point& max)
: min(min), max(max)
{
}

AABB::AABB(const Polygons& polys)
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(polys);
}

void AABB::calculate(const Polygons& polys)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for(unsigned int i=0; i<polys.size(); i++)
    {
        for(unsigned int j=0; j<polys[i].size(); j++)
        {
            include(polys[i][j]);
        }
    }
}

bool AABB::hit(const AABB& other) const
{
    if (max.X < other.min.X) return false;
    if (min.X > other.max.X) return false;
    if (max.Y < other.min.Y) return false;
    if (min.Y > other.max.Y) return false;
    return true;
}

void AABB::include(Point point)
{
    min.X = std::min(min.X,point.X);
    min.Y = std::min(min.Y,point.Y);
    max.X = std::max(max.X,point.X);
    max.Y = std::max(max.Y,point.Y);
}

void AABB::expand(int dist)
{
    if (min == Point(POINT_MAX, POINT_MAX) || max == Point(POINT_MIN, POINT_MIN))
    {
        return;
    }
    min.X -= dist;
    min.Y -= dist;
    max.X += dist;
    max.Y += dist;
}

}//namespace cura

