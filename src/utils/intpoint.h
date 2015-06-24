/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INT_POINT_H
#define INT_POINT_H

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <clipper/clipper.hpp>

#include <limits>
#include <stdint.h>
#include <cmath>

#include <functional> // for hash function obkject

#include <iostream> // auto-serialization / auto-toString()

#define INT2MM(n) (double(n) / 1000.0)
#define INT2MM2(n) (double(n) / 1000000.0)
#define MM2INT(n) (int64_t((n) * 1000))
#define MM2_2INT(n) (int64_t((n) * 1000000))
#define MM3_2INT(n) (int64_t((n) * 1000000000))

#define INT2MICRON(n) ((n) / 1)
#define MICRON2INT(n) ((n) * 1)

//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

class Point3
{
public:
    int32_t x,y,z;
    Point3() {}
    Point3(const int32_t _x, const int32_t _y, const int32_t _z): x(_x), y(_y), z(_z) {}

    Point3 operator+(const Point3& p) const { return Point3(x+p.x, y+p.y, z+p.z); }
    Point3 operator-(const Point3& p) const { return Point3(x-p.x, y-p.y, z-p.z); }
    Point3 operator/(const int32_t i) const { return Point3(x/i, y/i, z/i); }
    Point3 operator*(const int32_t i) const { return Point3(x*i, y*i, z*i); }
    Point3 operator*(const double d) const { return Point3(d*x, d*y, d*z); }

    Point3& operator += (const Point3& p) { x += p.x; y += p.y; z += p.z; return *this; }
    Point3& operator -= (const Point3& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }

    bool operator==(const Point3& p) const { return x==p.x&&y==p.y&&z==p.z; }
    bool operator!=(const Point3& p) const { return x!=p.x||y!=p.y||z!=p.z; }


    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const Point3& p)
    {
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }


    int32_t max()
    {
        if (x > y && x > z) return x;
        if (y > z) return y;
        return z;
    }

    bool testLength(int32_t len)
    {
        if (x > len || x < -len)
            return false;
        if (y > len || y < -len)
            return false;
        if (z > len || z < -len)
            return false;
        return vSize2() <= len*len;
    }

    int64_t vSize2() const
    {
        return int64_t(x)*int64_t(x)+int64_t(y)*int64_t(y)+int64_t(z)*int64_t(z);
    }

    int32_t vSize() const
    {
        return sqrt(vSize2());
    }
    
    double vSizeMM() const
    {
        double fx = INT2MM(x);
        double fy = INT2MM(y);
        double fz = INT2MM(z);
        return sqrt(fx*fx+fy*fy+fz*fz);
    }
    /*! this function is deprecated because it can cause overflows for vectors which easily fit inside a printer. Use FPoint3.cross(a,b) instead. */
    DEPRECATED(Point3 cross(const Point3& p))
    {
        return Point3(
            y*p.z-z*p.y, /// dangerous for vectors longer than 4.6 cm !!!!!
            z*p.x-x*p.z, /// can cause overflows
            x*p.y-y*p.x);
    }

    int64_t dot(const Point3& p)
    {
        return x*p.x + y*p.y + z*p.z;
    }

};

inline Point3 operator*(const int32_t i, const Point3& rhs) {
    return rhs * i;
}

inline Point3 operator*(const double d, const Point3& rhs) {
    return rhs * d;
}

/* 64bit Points are used mostly troughout the code, these are the 2D points from ClipperLib */
typedef ClipperLib::IntPoint Point;

class IntPoint {
public:
    int X, Y;
    Point p() { return Point(X, Y); }
};
#define POINT_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINT_MAX std::numeric_limits<ClipperLib::cInt>::max()

/* Extra operators to make it easier to do math with the 64bit Point objects */
INLINE Point operator-(const Point& p0) { return Point(-p0.X, -p0.Y); }
INLINE Point operator+(const Point& p0, const Point& p1) { return Point(p0.X+p1.X, p0.Y+p1.Y); }
INLINE Point operator-(const Point& p0, const Point& p1) { return Point(p0.X-p1.X, p0.Y-p1.Y); }
INLINE Point operator*(const Point& p0, const int32_t i) { return Point(p0.X*i, p0.Y*i); }
INLINE Point operator*(const int32_t i, const Point& p0) { return p0 * i; }
INLINE Point operator/(const Point& p0, const int32_t i) { return Point(p0.X/i, p0.Y/i); }

INLINE Point& operator += (Point& p0, const Point& p1) { p0.X += p1.X; p0.Y += p1.Y; return p0; }
INLINE Point& operator -= (Point& p0, const Point& p1) { p0.X -= p1.X; p0.Y -= p1.Y; return p0; }

//INLINE bool operator==(const Point& p0, const Point& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
//INLINE bool operator!=(const Point& p0, const Point& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

INLINE int64_t vSize2(const Point& p0)
{
    return p0.X*p0.X+p0.Y*p0.Y;
}
INLINE float vSize2f(const Point& p0)
{
    return float(p0.X)*float(p0.X)+float(p0.Y)*float(p0.Y);
}

INLINE bool shorterThen(const Point& p0, int32_t len)
{
    if (p0.X > len || p0.X < -len)
        return false;
    if (p0.Y > len || p0.Y < -len)
        return false;
    return vSize2(p0) <= len*len;
}

INLINE int64_t vSize(const Point& p0)
{
    return sqrt(vSize2(p0));
}

INLINE double vSizeMM(const Point& p0)
{
    double fx = INT2MM(p0.X);
    double fy = INT2MM(p0.Y);
    return sqrt(fx*fx+fy*fy);
}

INLINE Point normal(const Point& p0, int64_t len)
{
    int64_t _len = vSize(p0);
    if (_len < 1)
        return Point(len, 0);
    return p0 * len / _len;
}

INLINE Point crossZ(const Point& p0)
{
    return Point(-p0.Y, p0.X);
}
INLINE int64_t dot(const Point& p0, const Point& p1)
{
    return p0.X * p1.X + p0.Y * p1.Y;
}

INLINE int angle(const Point& p)
{
    double angle = std::atan2(p.X, p.Y) / M_PI * 180.0;
    if (angle < 0.0) angle += 360.0;
    return angle;
}

namespace std {
template <>
struct hash<Point> {
    size_t operator()(const Point & pp) const
    {
        static int prime = 31;
        int result = 89;
        result = result * prime + pp.X;
        result = result * prime + pp.Y;
        return result; 
    }
};
}

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

#endif//INT_POINT_H
