// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_INT_POINT_H
#define UTILS_INT_POINT_H

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

// Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <cmath>
#include <functional> // for hash function object
#include <iostream> // auto-serialization / auto-toString()
#include <limits>
#include <polyclipping/clipper.hpp>
#include <stdint.h>

#include "../utils/math.h" // for PI. Use relative path to avoid pulling <math.h>
#include "Point3LL.h" //For applying Point3Matrices.

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

/* 64bit Points are used mostly throughout the code, these are the 2D points from ClipperLib */
typedef ClipperLib::IntPoint Point2LL;

#define POINT_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINT_MAX std::numeric_limits<ClipperLib::cInt>::max()

static Point2LL no_point(std::numeric_limits<ClipperLib::cInt>::min(), std::numeric_limits<ClipperLib::cInt>::min());

/* Extra operators to make it easier to do math with the 64bit Point objects */
INLINE Point2LL operator-(const Point2LL& p0)
{
    return Point2LL(-p0.X, -p0.Y);
}
INLINE Point2LL operator+(const Point2LL& p0, const Point2LL& p1)
{
    return Point2LL(p0.X + p1.X, p0.Y + p1.Y);
}
INLINE Point2LL operator-(const Point2LL& p0, const Point2LL& p1)
{
    return Point2LL(p0.X - p1.X, p0.Y - p1.Y);
}
INLINE Point2LL operator*(const Point2LL& p0, const coord_t i)
{
    return Point2LL(p0.X * i, p0.Y * i);
}
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> // Use only for numeric types.
INLINE Point2LL operator*(const Point2LL& p0, const T i)
{
    return Point2LL(std::llrint(static_cast<T>(p0.X) * i), std::llrint(static_cast<T>(p0.Y) * i));
}
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> // Use only for numeric types.
INLINE Point2LL operator*(const T i, const Point2LL& p0)
{
    return p0 * i;
}
INLINE Point2LL operator/(const Point2LL& p0, const coord_t i)
{
    return Point2LL(p0.X / i, p0.Y / i);
}
INLINE Point2LL operator/(const Point2LL& p0, const Point2LL& p1)
{
    return Point2LL(p0.X / p1.X, p0.Y / p1.Y);
}
INLINE Point2LL operator%(const Point2LL& p0, const coord_t i)
{
    return Point2LL(p0.X % i, p0.Y % i);
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
    return sqrt(fx * fx + fy * fy);
}

INLINE Point2LL normal(const Point2LL& p0, coord_t len)
{
    coord_t _len = vSize(p0);
    if (_len < 1)
        return Point2LL(len, 0);
    return p0 * len / _len;
}

INLINE Point2LL turn90CCW(const Point2LL& p0)
{
    return Point2LL(-p0.Y, p0.X);
}

INLINE Point2LL rotate(const Point2LL& p0, double angle)
{
    const double cos_component = std::cos(angle);
    const double sin_component = std::sin(angle);
    const double x = static_cast<double>(p0.X);
    const double y = static_cast<double>(p0.Y);
    return Point2LL(std::llrint(cos_component * x - sin_component * y), std::llrint(sin_component * x + cos_component * y));
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
        angle += 360.0;
    return angle;
}

// Identity function, used to be able to make templated algorithms where the input is sometimes points, sometimes things that contain or can be converted to points.
INLINE const Point2LL& make_point(const Point2LL& p)
{
    return p;
}

} // namespace cura

namespace std
{
template<>
struct hash<cura::Point2LL>
{
    size_t operator()(const cura::Point2LL& pp) const
    {
        static int prime = 31;
        int result = 89;
        result = static_cast<int>(result * prime + pp.X);
        result = static_cast<int>(result * prime + pp.Y);
        return static_cast<size_t>(result);
    }
};
} // namespace std

namespace cura
{

class PointMatrix
{
public:
    double matrix[4];

    PointMatrix()
    {
        matrix[0] = 1;
        matrix[1] = 0;
        matrix[2] = 0;
        matrix[3] = 1;
    }

    PointMatrix(double rotation)
    {
        rotation = rotation / 180 * std::numbers::pi;
        matrix[0] = cos(rotation);
        matrix[1] = -sin(rotation);
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    PointMatrix(const Point2LL p)
    {
        matrix[0] = static_cast<double>(p.X);
        matrix[1] = static_cast<double>(p.Y);
        double f = sqrt((matrix[0] * matrix[0]) + (matrix[1] * matrix[1]));
        matrix[0] /= f;
        matrix[1] /= f;
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    static PointMatrix scale(double s)
    {
        PointMatrix ret;
        ret.matrix[0] = s;
        ret.matrix[3] = s;
        return ret;
    }

    Point2LL apply(const Point2LL p) const
    {
        const double x = static_cast<double>(p.X);
        const double y = static_cast<double>(p.Y);
        return Point2LL(std::llrint(x * matrix[0] + y * matrix[1]), std::llrint(x * matrix[2] + y * matrix[3]));
    }

    /*!
     * \warning only works on a rotation matrix! Output is incorrect for other types of matrix
     */
    Point2LL unapply(const Point2LL p) const
    {
        const double x = static_cast<double>(p.X);
        const double y = static_cast<double>(p.Y);
        return Point2LL(std::llrint(x * matrix[0] + y * matrix[2]), std::llrint(x * matrix[1] + y * matrix[3]));
    }

    PointMatrix inverse() const
    {
        PointMatrix ret;
        double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        ret.matrix[0] = matrix[3] / det;
        ret.matrix[1] = -matrix[1] / det;
        ret.matrix[2] = -matrix[2] / det;
        ret.matrix[3] = matrix[0] / det;
        return ret;
    }
};

class Point3Matrix
{
public:
    double matrix[9];

    Point3Matrix()
    {
        matrix[0] = 1;
        matrix[1] = 0;
        matrix[2] = 0;
        matrix[3] = 0;
        matrix[4] = 1;
        matrix[5] = 0;
        matrix[6] = 0;
        matrix[7] = 0;
        matrix[8] = 1;
    }

    /*!
     * Initializes the top left corner with the values of \p b
     * and the rest as if it's a unit matrix
     */
    Point3Matrix(const PointMatrix& b)
    {
        matrix[0] = b.matrix[0];
        matrix[1] = b.matrix[1];
        matrix[2] = 0;
        matrix[3] = b.matrix[2];
        matrix[4] = b.matrix[3];
        matrix[5] = 0;
        matrix[6] = 0;
        matrix[7] = 0;
        matrix[8] = 1;
    }

    Point3LL apply(const Point3LL p) const
    {
        const double x = static_cast<double>(p.x_);
        const double y = static_cast<double>(p.y_);
        const double z = static_cast<double>(p.z_);
        return Point3LL(
            std::llrint(x * matrix[0] + y * matrix[1] + z * matrix[2]),
            std::llrint(x * matrix[3] + y * matrix[4] + z * matrix[5]),
            std::llrint(x * matrix[6] + y * matrix[7] + z * matrix[8]));
    }

    /*!
     * Apply matrix to vector as homogeneous coordinates.
     */
    Point2LL apply(const Point2LL p) const
    {
        Point3LL result = apply(Point3LL(p.X, p.Y, 1));
        return Point2LL(result.x_ / result.z_, result.y_ / result.z_);
    }

    static Point3Matrix translate(const Point2LL p)
    {
        Point3Matrix ret; // uniform matrix
        ret.matrix[2] = static_cast<double>(p.X);
        ret.matrix[5] = static_cast<double>(p.Y);
        return ret;
    }

    Point3Matrix compose(const Point3Matrix& b)
    {
        Point3Matrix ret;
        for (int outx = 0; outx < 3; outx++)
        {
            for (int outy = 0; outy < 3; outy++)
            {
                ret.matrix[outy * 3 + outx] = 0;
                for (int in = 0; in < 3; in++)
                {
                    ret.matrix[outy * 3 + outx] += matrix[outy * 3 + in] * b.matrix[in * 3 + outx];
                }
            }
        }
        return ret;
    }
};


inline Point3LL operator+(const Point3LL& p3, const Point2LL& p2)
{
    return Point3LL(p3.x_ + p2.X, p3.y_ + p2.Y, p3.z_);
}
inline Point3LL& operator+=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ += p2.X;
    p3.y_ += p2.Y;
    return p3;
}

inline Point2LL operator+(const Point2LL& p2, const Point3LL& p3)
{
    return Point2LL(p3.x_ + p2.X, p3.y_ + p2.Y);
}


inline Point3LL operator-(const Point3LL& p3, const Point2LL& p2)
{
    return Point3LL(p3.x_ - p2.X, p3.y_ - p2.Y, p3.z_);
}
inline Point3LL& operator-=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ -= p2.X;
    p3.y_ -= p2.Y;
    return p3;
}

inline Point2LL operator-(const Point2LL& p2, const Point3LL& p3)
{
    return Point2LL(p2.X - p3.x_, p2.Y - p3.y_);
}

} // namespace cura
#endif // UTILS_INT_POINT_H
