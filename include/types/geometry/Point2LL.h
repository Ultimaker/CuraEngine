// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#include <cmath>
#include <limits>
#include <polyclipping/clipper.hpp>

#include "types/AngleDegrees.h"
#include "types/AngleRadians.h"
#include "types/Coord_t.h"
#include "types/generic.h"

namespace curaengine::types::geometry
{

using types::fuzzy_equal;
using types::lerp;

class Point3LL;

/* 64bit Points are used mostly throughout the code, these are the 2D points from ClipperLib */
using Point2LL = ClipperLib::IntPoint;

inline constexpr coord_t point_min = std::numeric_limits<coord_t>::min();
inline constexpr coord_t point_max = std::numeric_limits<coord_t>::max();

static Point2LL no_point(point_min, point_min);

/* Extra operators to make it easier to do math with the 64bit Point objects */
static inline Point2LL operator-(const Point2LL& p0)
{
    return { -p0.X, -p0.Y };
}

static inline Point2LL operator+(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X + p1.X, p0.Y + p1.Y };
}

static inline Point2LL operator-(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X - p1.X, p0.Y - p1.Y };
}

static inline Point2LL operator*(const Point2LL& p0, const types::coord_t i)
{
    return { p0.X * i, p0.Y * i };
}

template<types::numeric T> // Use only for numeric types.
static inline Point2LL operator*(const Point2LL& p0, const T i)
{
    return { std::llrint(static_cast<T>(p0.X) * i), std::llrint(static_cast<T>(p0.Y) * i) };
}

template<types::numeric T>
static inline Point2LL operator*(const T i, const Point2LL& p0)
{
    return p0 * i;
}

static inline Point2LL operator/(const Point2LL& p0, const types::coord_t i)
{
    return { p0.X / i, p0.Y / i };
}

static inline Point2LL operator/(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X / p1.X, p0.Y / p1.Y };
}

static inline Point2LL operator%(const Point2LL& p0, const types::coord_t i)
{
    return { p0.X % i, p0.Y % i };
}

static inline Point2LL& operator+=(Point2LL& p0, const Point2LL& p1)
{
    p0.X += p1.X;
    p0.Y += p1.Y;
    return p0;
}

static inline Point2LL& operator-=(Point2LL& p0, const Point2LL& p1)
{
    p0.X -= p1.X;
    p0.Y -= p1.Y;
    return p0;
}

/* ***** NOTE *****
   TL;DR: DO NOT implement operators *= and /= because of the default values in ClipperLib::IntPoint's constructor.

   We DO NOT implement operators *= and /= because the class Point is essentially a ClipperLib::IntPoint and it has a
   constructor IntPoint(int x = 0, int y = 0), and this causes problems. If you implement *= as *=(int) and when you
   do "Point a = a * 5", you probably intend to do "a.x *= 5" and "a.y *= 5", but with that constructor, it will create
   an IntPoint(5, y = 0) and you end up with wrong results.
 */

// static inline bool operator==(const Point& p0, const Point& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
// static inline bool operator!=(const Point& p0, const Point& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

static inline types::coord_t vSize2(const Point2LL& p0)
{
    return p0.X * p0.X + p0.Y * p0.Y;
}

static inline double vSize2f(const Point2LL& p0)
{
    return static_cast<double>(p0.X) * static_cast<double>(p0.X) + static_cast<double>(p0.Y) * static_cast<double>(p0.Y);
}

static inline bool shorterThan(const Point2LL& p0, const types::coord_t len)
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

static inline types::coord_t vSize(const Point2LL& p0)
{
    return std::llrint(sqrt(static_cast<double>(vSize2(p0))));
}

static inline types::coord_t vSizef(const Point2LL& p0)
{
    return std::sqrt(vSize2f(p0));
}

static inline double vSizeMM(const Point2LL& p0)
{
    double fx = intToMm(p0.X);
    double fy = intToMm(p0.Y);
    return std::sqrt(fx * fx + fy * fy);
}

static inline Point2LL normal(const Point2LL& p0, types::coord_t length)
{
    const types::coord_t len{ vSize(p0) };
    if (len < 1)
    {
        return { length, 0 };
    }
    return p0 * length / len;
}

static inline Point2LL turn90CCW(const Point2LL& p0)
{
    return { -p0.Y, p0.X };
}

static inline Point2LL rotate(const Point2LL& p0, double angle)
{
    const double cos_component = std::cos(angle);
    const double sin_component = std::sin(angle);
    const auto x = static_cast<double>(p0.X);
    const auto y = static_cast<double>(p0.Y);
    return { std::llrint(cos_component * x - sin_component * y), std::llrint(sin_component * x + cos_component * y) };
}

static inline types::coord_t dot(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.X + p0.Y * p1.Y;
}

static inline types::coord_t cross(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.Y - p0.Y * p1.X;
}

// Identity function, used to be able to make templated algorithms where the input is sometimes points, sometimes things that contain or can be converted to points.
static inline const Point2LL& make_point(const Point2LL& p)
{
    return p;
}

static inline const types::AngleRadians angle_rad(const Point2LL& p)
{
    return std::atan2(p.Y, p.X);
}

static inline const types::AngleDegrees angle_deg(const Point2LL& p)
{
    return types::AngleDegrees(angle_rad(p));
}

Point2LL operator+(const Point2LL& p2, const Point3LL& p3);

Point3LL operator+(const Point3LL& p3, const Point2LL& p2);

Point3LL& operator+=(Point3LL& p3, const Point2LL& p2);

Point3LL operator-(const Point3LL& p3, const Point2LL& p2);

Point3LL& operator-=(Point3LL& p3, const Point2LL& p2);

Point2LL operator-(const Point2LL& p2, const Point3LL& p3);

inline Point2LL lerp(const Point2LL& a, const Point2LL& b, const double t)
{
    return Point2LL(lerp(a.X, b.X, t), lerp(a.Y, b.Y, t));
}

/*! Returns true if the points are equal or close enough to each other to be considered equal */
inline bool fuzzy_equal(const Point2LL& a, const Point2LL& b)
{
    return fuzzy_equal(a.X, b.X) && fuzzy_equal(a.Y, b.Y);
}

} // namespace curaengine::types::geometry

namespace std
{
template<>
struct hash<::curaengine::types::geometry::Point2LL>
{
    size_t operator()(const ::curaengine::types::geometry::Point2LL& pp) const noexcept
    {
        static int prime = 31;
        int result = 89;
        result = static_cast<int>(result * prime + pp.X);
        result = static_cast<int>(result * prime + pp.Y);
        return static_cast<size_t>(result);
    }
};
} // namespace std

namespace ClipperLib
{
using ::curaengine::types::geometry::operator+;
using ::curaengine::types::geometry::operator-;
using ::curaengine::types::geometry::operator*;
using ::curaengine::types::geometry::operator/;
using ::curaengine::types::geometry::operator%;
using ::curaengine::types::geometry::operator+=;
using ::curaengine::types::geometry::operator-=;

inline bool operator<(const IntPoint& lhs, const IntPoint& rhs)
{
    return lhs.X < rhs.X || (lhs.X == rhs.X && lhs.Y < rhs.Y);
}
} // namespace ClipperLib
