/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "AABB.h"

#include <limits>

namespace cura
{


AABB::AABB()
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
}

AABB::AABB(const Point& min, const Point& max)
: min(min), max(max)
{
}

AABB::AABB(const Polygons& polys)
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(polys);
}

AABB::AABB(ConstPolygonRef poly)
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(poly);
}

Point AABB::getMiddle() const
{
    return (min + max) / 2;
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

void AABB::calculate(ConstPolygonRef poly)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for (const Point& p : poly)
    {
        include(p);
    }
}

bool AABB::contains(const Point& point) const
{
    return point.X >= min.X && point.X <= max.X && point.Y >= min.Y && point.Y <= max.Y;
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

void AABB::include(const AABB other)
{
    include(other.min);
    include(other.max);
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

void AABB::round(const coord_t increment)
{
    if (increment <= 1) //If rounding to single microns we don't have to do anything. Anything lower than that doesn't make sense so ignore that.
    {
        return;
    }
    min.X = (min.X - (min.X >= 0) * (increment - 1)) / increment * increment; //Round the min vector towards negative.
    min.Y = (min.Y - (min.Y >= 0) * (increment - 1)) / increment * increment;
    max.X = (max.X + (max.X >= 0) * (increment - 1)) / increment * increment; //Round the max vector towards positive.
    max.Y = (max.Y + (max.Y >= 0) * (increment - 1)) / increment * increment;
}

Polygon AABB::toPolygon() const
{
    Polygon ret;
    ret.add(min);
    ret.add(Point(max.X, min.Y));
    ret.add(max);
    ret.add(Point(min.X, max.Y));
    return ret;
}

}//namespace cura

