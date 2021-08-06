//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_INT_POINT_H
#define UTILS_INT_POINT_H

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <polyclipping/clipper.hpp>
#include <cmath>
#include <functional> // for hash function object
#include <iostream> // auto-serialization / auto-toString()
#include <limits>
#include <stdint.h>

#include "Point3.h" //For applying Point3Matrices.


#include "../utils/math.h" // for M_PI. Use relative path to avoid pulling <math.h>

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


namespace cura
{

/* 64bit Points are used mostly throughout the code, these are the 2D points from ClipperLib */
typedef ClipperLib::IntPoint Point;

class IntPoint {
public:
    int X, Y;
    Point p() { return Point(X, Y); }
};
#define POINT_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINT_MAX std::numeric_limits<ClipperLib::cInt>::max()

static Point no_point(std::numeric_limits<ClipperLib::cInt>::min(), std::numeric_limits<ClipperLib::cInt>::min());

/* Extra operators to make it easier to do math with the 64bit Point objects */
INLINE Point operator-(const Point& p0) { return Point(-p0.X, -p0.Y); }
INLINE Point operator+(const Point& p0, const Point& p1) { return Point(p0.X+p1.X, p0.Y+p1.Y); }
INLINE Point operator-(const Point& p0, const Point& p1) { return Point(p0.X-p1.X, p0.Y-p1.Y); }
INLINE Point operator*(const Point& p0, const coord_t i) { return Point(p0.X * i, p0.Y * i); }
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> //Use only for numeric types.
INLINE Point operator*(const Point& p0, const T i) { return Point(p0.X * i, p0.Y * i); }
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> //Use only for numeric types.
INLINE Point operator*(const T i, const Point& p0) { return p0 * i; }
INLINE Point operator/(const Point& p0, const coord_t i) { return Point(p0.X/i, p0.Y/i); }
INLINE Point operator/(const Point& p0, const Point& p1) { return Point(p0.X/p1.X, p0.Y/p1.Y); }

INLINE Point& operator += (Point& p0, const Point& p1) { p0.X += p1.X; p0.Y += p1.Y; return p0; }
INLINE Point& operator -= (Point& p0, const Point& p1) { p0.X -= p1.X; p0.Y -= p1.Y; return p0; }

/* ***** NOTE *****
   TL;DR: DO NOT implement operators *= and /= because of the default values in ClipperLib::IntPoint's constructor.

   We DO NOT implement operators *= and /= because the class Point is essentially a ClipperLib::IntPoint and it has a
   constructor IntPoint(int x = 0, int y = 0), and this causes problems. If you implement *= as *=(int) and when you
   do "Point a = a * 5", you probably intend to do "a.x *= 5" and "a.y *= 5", but with that constructor, it will create
   an IntPoint(5, y = 0) and you end up with wrong results.
 */

//INLINE bool operator==(const Point& p0, const Point& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
//INLINE bool operator!=(const Point& p0, const Point& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

INLINE coord_t vSize2(const Point& p0)
{
    return p0.X*p0.X+p0.Y*p0.Y;
}
INLINE float vSize2f(const Point& p0)
{
    return static_cast<float>(p0.X)*static_cast<float>(p0.X)+static_cast<float>(p0.Y)*static_cast<float>(p0.Y);
}

INLINE bool shorterThen(const Point& p0, const coord_t len)
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

INLINE coord_t vSize(const Point& p0)
{
    return sqrt(vSize2(p0));
}

INLINE double vSizeMM(const Point& p0)
{
    double fx = INT2MM(p0.X);
    double fy = INT2MM(p0.Y);
    return sqrt(fx*fx+fy*fy);
}

INLINE Point normal(const Point& p0, coord_t len)
{
    coord_t _len = vSize(p0);
    if (_len < 1)
        return Point(len, 0);
    return p0 * len / _len;
}

INLINE Point turn90CCW(const Point& p0)
{
    return Point(-p0.Y, p0.X);
}

INLINE Point rotate(const Point& p0, double angle)
{
    const double cos_component = std::cos(angle);
    const double sin_component = std::sin(angle);
    return Point(cos_component * p0.X - sin_component * p0.Y, sin_component * p0.X + cos_component * p0.Y);
}

INLINE coord_t dot(const Point& p0, const Point& p1)
{
    return p0.X * p1.X + p0.Y * p1.Y;
}

INLINE int angle(const Point& p)
{
    double angle = std::atan2(p.X, p.Y) / M_PI * 180.0;
    if (angle < 0.0) angle += 360.0;
    return angle;
}

}//namespace cura

namespace std {
template <>
struct hash<cura::Point> {
    size_t operator()(const cura::Point & pp) const
    {
        static int prime = 31;
        int result = 89;
        result = result * prime + pp.X;
        result = result * prime + pp.Y;
        return result;
    }
};
}

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
        rotation = rotation / 180 * M_PI;
        matrix[0] = cos(rotation);
        matrix[1] = -sin(rotation);
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    PointMatrix(const Point p)
    {
        matrix[0] = p.X;
        matrix[1] = p.Y;
        double f = sqrt((matrix[0] * matrix[0]) + (matrix[1] * matrix[1]));
        matrix[0] /= f;
        matrix[1] /= f;
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    Point apply(const Point p) const
    {
        return Point(p.X * matrix[0] + p.Y * matrix[1], p.X * matrix[2] + p.Y * matrix[3]);
    }

    Point unapply(const Point p) const
    {
        return Point(p.X * matrix[0] + p.Y * matrix[2], p.X * matrix[1] + p.Y * matrix[3]);
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

    Point3 apply(const Point3 p) const
    {
        return Point3(p.x * matrix[0] + p.y * matrix[1] + p.z * matrix[2]
                    , p.x * matrix[3] + p.y * matrix[4] + p.z * matrix[5]
                    , p.x * matrix[6] + p.y * matrix[7] + p.z * matrix[8]);
    }

    /*!
     * Apply matrix to vector as homogeneous coordinates.
     */
    Point apply(const Point p) const
    {
        Point3 result = apply(Point3(p.X, p.Y, 1));
        return Point(result.x / result.z, result.y / result.z);
    }

    static Point3Matrix translate(const Point p)
    {
        Point3Matrix ret; // uniform matrix
        ret.matrix[2] = p.X;
        ret.matrix[5] = p.Y;
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


inline Point3 operator+(const Point3& p3, const Point& p2) {
    return Point3(p3.x + p2.X, p3.y + p2.Y, p3.z);
}
inline Point3& operator+=(Point3& p3, const Point& p2) {
    p3.x += p2.X;
    p3.y += p2.Y;
    return p3;
}

inline Point operator+(const Point& p2, const Point3& p3) {
    return Point(p3.x + p2.X, p3.y + p2.Y);
}


inline Point3 operator-(const Point3& p3, const Point& p2) {
    return Point3(p3.x - p2.X, p3.y - p2.Y, p3.z);
}
inline Point3& operator-=(Point3& p3, const Point& p2) {
    p3.x -= p2.X;
    p3.y -= p2.Y;
    return p3;
}

inline Point operator-(const Point& p2, const Point3& p3) {
    return Point(p2.X - p3.x, p2.Y - p3.y);
}

}//namespace cura
#endif//UTILS_INT_POINT_H

