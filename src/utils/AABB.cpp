// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/AABB.h"

#include <limits>

#include "utils/linearAlg2D.h"
#include "utils/polygon.h" //To create the AABB of a polygon.

namespace cura
{


AABB::AABB()
    : min_(POINT_MAX, POINT_MAX)
    , max_(POINT_MIN, POINT_MIN)
{
}

AABB::AABB(const Point2LL& min, const Point2LL& max)
    : min_(min)
    , max_(max)
{
}

AABB::AABB(const Polygons& polys)
    : min_(POINT_MAX, POINT_MAX)
    , max_(POINT_MIN, POINT_MIN)
{
    calculate(polys);
}

AABB::AABB(ConstPolygonRef poly)
    : min_(POINT_MAX, POINT_MAX)
    , max_(POINT_MIN, POINT_MIN)
{
    calculate(poly);
}

Point2LL AABB::getMiddle() const
{
    return (min_ + max_) / 2;
}

coord_t AABB::distanceSquared(const Point2LL& p) const
{
    const Point2LL a = Point2LL(max_.X, min_.Y);
    const Point2LL b = Point2LL(min_.X, max_.Y);
    return (contains(p) ? -1 : 1)
         * std::min({ LinearAlg2D::getDist2FromLineSegment(min_, a, p),
                      LinearAlg2D::getDist2FromLineSegment(a, max_, p),
                      LinearAlg2D::getDist2FromLineSegment(max_, b, p),
                      LinearAlg2D::getDist2FromLineSegment(b, min_, p) });
}

coord_t AABB::distanceSquared(const AABB& other) const
{
    return std::min({
        distanceSquared(other.min_),
        other.distanceSquared(min_),
        distanceSquared(other.max_),
        other.distanceSquared(max_),
        distanceSquared(Point2LL(other.max_.X, other.min_.Y)),
        other.distanceSquared(Point2LL(max_.X, min_.Y)),
        distanceSquared(Point2LL(other.min_.X, other.max_.Y)),
        other.distanceSquared(Point2LL(min_.X, max_.Y)),
    });
}

void AABB::calculate(const Polygons& polys)
{
    min_ = Point2LL(POINT_MAX, POINT_MAX);
    max_ = Point2LL(POINT_MIN, POINT_MIN);
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
    min_ = Point2LL(POINT_MAX, POINT_MAX);
    max_ = Point2LL(POINT_MIN, POINT_MIN);
    for (const Point2LL& p : poly)
    {
        include(p);
    }
}

bool AABB::contains(const Point2LL& point) const
{
    return point.X >= min_.X && point.X <= max_.X && point.Y >= min_.Y && point.Y <= max_.Y;
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
    return other.min_.X >= min_.X && other.max_.X <= max_.X && other.min_.Y >= min_.Y && other.max_.Y <= max_.Y;
}

coord_t AABB::area() const
{
    if (max_.X < min_.X || max_.Y < min_.Y)
    {
        return -1;
    } // Do the unititialized check explicitly, so there aren't any problems with over/underflow and POINT_MAX/POINT_MIN.
    return (max_.X - min_.X) * (max_.Y - min_.Y);
}

bool AABB::hit(const AABB& other) const
{
    if (max_.X < other.min_.X)
        return false;
    if (min_.X > other.max_.X)
        return false;
    if (max_.Y < other.min_.Y)
        return false;
    if (min_.Y > other.max_.Y)
        return false;
    return true;
}

void AABB::include(Point2LL point)
{
    min_.X = std::min(min_.X, point.X);
    min_.Y = std::min(min_.Y, point.Y);
    max_.X = std::max(max_.X, point.X);
    max_.Y = std::max(max_.Y, point.Y);
}

void AABB::include(const AABB other)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min_.X = std::min(min_.X, other.min_.X);
    min_.Y = std::min(min_.Y, other.min_.Y);
    max_.X = std::max(max_.X, other.max_.X);
    max_.Y = std::max(max_.Y, other.max_.Y);
}

void AABB::expand(int dist)
{
    if (min_ == Point2LL(POINT_MAX, POINT_MAX) || max_ == Point2LL(POINT_MIN, POINT_MIN))
    {
        return;
    }
    min_.X -= dist;
    min_.Y -= dist;
    max_.X += dist;
    max_.Y += dist;
}

Polygon AABB::toPolygon() const
{
    Polygon ret;
    ret.add(min_);
    ret.add(Point2LL(max_.X, min_.Y));
    ret.add(max_);
    ret.add(Point2LL(min_.X, max_.Y));
    return ret;
}

} // namespace cura
