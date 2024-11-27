// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT2LL_H
#define GEOMETRY_POINT2LL_H

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

#include <cmath>
#include <limits>
#include <numbers>
#include <polyclipping/clipper.hpp>

#include "utils/Coord_t.h"
#include "utils/types/generic.h"

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


namespace cura
{

class Point3LL;

/* 64bit Points are used mostly throughout the code, these are the 2D points from ClipperLib */
using Point2LL = ClipperLib::IntPoint;

#define POINT_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINT_MAX std::numeric_limits<ClipperLib::cInt>::max()

static Point2LL no_point(std::numeric_limits<ClipperLib::cInt>::min(), std::numeric_limits<ClipperLib::cInt>::min());

/* Extra operators to make it easier to do math with the 64bit Point objects */
INLINE Point2LL operator-(const Point2LL& p0)
{
    return { -p0.X, -p0.Y };
}

INLINE Point2LL operator+(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X + p1.X, p0.Y + p1.Y };
}

INLINE Point2LL operator-(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X - p1.X, p0.Y - p1.Y };
}

INLINE Point2LL operator*(const Point2LL& p0, const coord_t i)
{
    return { p0.X * i, p0.Y * i };
}

template<utils::numeric T> // Use only for numeric types.
INLINE Point2LL operator*(const Point2LL& p0, const T i)
{
    return { std::llrint(static_cast<T>(p0.X) * i), std::llrint(static_cast<T>(p0.Y) * i) };
}

template<utils::numeric T>
INLINE Point2LL operator*(const T i, const Point2LL& p0)
{
    return p0 * i;
}

INLINE Point2LL operator/(const Point2LL& p0, const coord_t i)
{
    return { p0.X / i, p0.Y / i };
}

INLINE Point2LL operator/(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X / p1.X, p0.Y / p1.Y };
}

INLINE Point2LL operator%(const Point2LL& p0, const coord_t i)
{
    return { p0.X % i, p0.Y % i };
}

INLINE Point2LL& operator+=(Point2LL& p0, const Point2LL& p1)
{
    p0.X += p1.X;
    p0.Y += p1.Y;
    return p0;
}

INLINE Point2LL& operator-=(Point2LL& p0, const Point2LL& p1)
{
    p0.X -= p1.X;
    p0.Y -= p1.Y;
    return p0;
}

INLINE bool operator<(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X < p1.X || (p0.X == p1.X && p0.Y < p1.Y);
}

/* ***** NOTE *****
   TL;DR: DO NOT implement operators *= and /= because of the default values in ClipperLib::IntPoint's constructor.

   We DO NOT implement operators *= and /= because the class Point is essentially a ClipperLib::IntPoint and it has a
   constructor IntPoint(int x = 0, int y = 0), and this causes problems. If you implement *= as *=(int) and when you
   do "Point a = a * 5", you probably intend to do "a.x *= 5" and "a.y *= 5", but with that constructor, it will create
   an IntPoint(5, y = 0) and you end up with wrong results.
 */

// INLINE bool operator==(const Point& p0, const Point& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
// INLINE bool operator!=(const Point& p0, const Point& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

INLINE coord_t vSize2(const Point2LL& p0)
{
    return p0.X * p0.X + p0.Y * p0.Y;
}

INLINE double vSize2f(const Point2LL& p0)
{
    return static_cast<double>(p0.X) * static_cast<double>(p0.X) + static_cast<double>(p0.Y) * static_cast<double>(p0.Y);
}

INLINE bool shorterThen(const Point2LL& p0, const coord_t len)
{
    if (p0.X > len || p0.X < -len)
    {
        return false;
    }
    if (p0.Y > len || p0.Y < -len)
    {
        return false;
    }
    return vSize2(p0) <= len * len;
}

INLINE bool shorterThan(const Point2LL& p0, const coord_t len)
{
    return shorterThen(p0, len);
}

INLINE coord_t vSize(const Point2LL& p0)
{
    return std::llrint(sqrt(static_cast<double>(vSize2(p0))));
}

INLINE double vSizeMM(const Point2LL& p0)
{
    double fx = INT2MM(p0.X);
    double fy = INT2MM(p0.Y);
    return std::sqrt(fx * fx + fy * fy);
}

INLINE Point2LL normal(const Point2LL& p0, coord_t length)
{
    const coord_t len{ vSize(p0) };
    if (len < 1)
    {
        return { length, 0 };
    }
    return p0 * length / len;
}

INLINE Point2LL turn90CCW(const Point2LL& p0)
{
    return { -p0.Y, p0.X };
}

INLINE Point2LL rotate(const Point2LL& p0, double angle)
{
    const double cos_component = std::cos(angle);
    const double sin_component = std::sin(angle);
    const auto x = static_cast<double>(p0.X);
    const auto y = static_cast<double>(p0.Y);
    return { std::llrint(cos_component * x - sin_component * y), std::llrint(sin_component * x + cos_component * y) };
}

INLINE coord_t dot(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.X + p0.Y * p1.Y;
}

INLINE coord_t cross(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.Y - p0.Y * p1.X;
}

INLINE double angle(const Point2LL& p)
{
    double angle = std::atan2(p.X, p.Y) / std::numbers::pi * 180.0;
    if (angle < 0.0)
    {
        angle += 360.0;
    }
    return angle;
}

// Identity function, used to be able to make templated algorithms where the input is sometimes points, sometimes things that contain or can be converted to points.
INLINE const Point2LL& make_point(const Point2LL& p)
{
    return p;
}

Point2LL operator+(const Point2LL& p2, const Point3LL& p3);

Point3LL operator+(const Point3LL& p3, const Point2LL& p2);

Point3LL& operator+=(Point3LL& p3, const Point2LL& p2);

Point3LL operator-(const Point3LL& p3, const Point2LL& p2);

Point3LL& operator-=(Point3LL& p3, const Point2LL& p2);

Point2LL operator-(const Point2LL& p2, const Point3LL& p3);

} // namespace cura

namespace std
{
template<>
struct hash<cura::Point2LL>
{
    size_t operator()(const cura::Point2LL& pp) const noexcept
    {
        static int prime = 31;
        int result = 89;
        result = static_cast<int>(result * prime + pp.X);
        result = static_cast<int>(result * prime + pp.Y);
        return static_cast<size_t>(result);
    }
};
} // namespace std

#endif // GEOMETRY_POINT2LL_H
