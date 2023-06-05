// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/AABB.h"
#include "utils/linearAlg2D.h"
#include "utils/polygon.h" //To create the AABB of a polygon.
#include <limits>

namespace cura
{


AABB::AABB() : min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
}

AABB::AABB(const Point& min, const Point& max) : min(min), max(max)
{
}

AABB::AABB(const Polygons& polys) : min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(polys);
}

AABB::AABB(ConstPolygonRef poly) : min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(poly);
}

Point AABB::getMiddle() const
{
    return (min + max) / 2;
}

coord_t AABB::distanceSquared(const Point& p) const
{
    const Point a = Point(max.X, min.Y);
    const Point b = Point(min.X, max.Y);
    return (contains(p) ? -1 : 1)
         * std::min({ LinearAlg2D::getDist2FromLineSegment(min, a, p), LinearAlg2D::getDist2FromLineSegment(a, max, p), LinearAlg2D::getDist2FromLineSegment(max, b, p), LinearAlg2D::getDist2FromLineSegment(b, min, p) });
}

coord_t AABB::distanceSquared(const AABB& other) const
{
    return std::min({
        distanceSquared(other.min),
        other.distanceSquared(min),
        distanceSquared(other.max),
        other.distanceSquared(max),
        distanceSquared(Point(other.max.X, other.min.Y)),
        other.distanceSquared(Point(max.X, min.Y)),
        distanceSquared(Point(other.min.X, other.max.Y)),
        other.distanceSquared(Point(min.X, max.Y)),
    });
}

void AABB::calculate(const Polygons& polys)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for (unsigned int i = 0; i < polys.size(); i++)
    {
        for (unsigned int j = 0; j < polys[i].size(); j++)
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

bool AABB::contains(const AABB& other) const
{
    if (area() < 0)
    {
        return false;
    }
    if (other.area() < 0)
    {
        return true;
    }
    return other.min.X >= min.X && other.max.X <= max.X && other.min.Y >= min.Y && other.max.Y <= max.Y;
}

coord_t AABB::area() const
{
    if (max.X < min.X || max.Y < min.Y)
    {
        return -1;
    } // Do the unititialized check explicitly, so there aren't any problems with over/underflow and POINT_MAX/POINT_MIN.
    return (max.X - min.X) * (max.Y - min.Y);
}

bool AABB::hit(const AABB& other) const
{
    if (max.X < other.min.X)
        return false;
    if (min.X > other.max.X)
        return false;
    if (max.Y < other.min.Y)
        return false;
    if (min.Y > other.max.Y)
        return false;
    return true;
}

void AABB::include(Point point)
{
    min.X = std::min(min.X, point.X);
    min.Y = std::min(min.Y, point.Y);
    max.X = std::max(max.X, point.X);
    max.Y = std::max(max.Y, point.Y);
}

void AABB::include(const AABB other)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min.X = std::min(min.X, other.min.X);
    min.Y = std::min(min.Y, other.min.Y);
    max.X = std::max(max.X, other.max.X);
    max.Y = std::max(max.Y, other.max.Y);
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

coord_t AABB::distanceSquared(const cura::Point& p) const
{
    const Point lt = min;
    const Point rt = Point(max.X, min.Y);
    const Point rb = max;
    const Point lb = Point(min.X, max.Y);

    if (contains(p))
    {
        return 0;
    }
    else if (p.X < lt.X && p.Y < lt.Y)
    {
        return vSize2(p - lt);
    }
    else if (p.X > rt.X && p.Y < rt.Y)
    {
        return vSize2(p - rt);
    }
    else if (p.X > rb.X && p.Y > rb.Y)
    {
        return vSize2(p - rb);
    }
    else if (p.X < lb.X && p.Y > lb.Y)
    {
        return vSize2(p - lb);
    }
    else if (p.X < min.X)
    {
        const auto dist = min.X - p.X;
        return dist * dist;
    }
    else if (p.X > max.X)
    {
        const auto dist = p.X - max.X;
        return dist * dist;
    }
    else if (p.Y < min.Y)
    {
        const auto dist = min.Y - p.Y;
        return dist * dist;
    }
    else if (p.Y > max.Y)
    {
        const auto dist = p.Y - max.Y;
        return dist * dist;
    }
    else
    {
        assert(false && "Not a possible state");
    }
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

} // namespace cura
