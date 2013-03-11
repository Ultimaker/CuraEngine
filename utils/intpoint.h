#ifndef INT_POINT_H
#define INT_POINT_H

/*
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

#include "clipper/clipper.hpp"
using ClipperLib::Polygons;

#include <limits.h>
#include <stdint.h>
#include <math.h>

class Point3
{
public:
    int32_t x,y,z;
    Point3() {}
    Point3(int32_t _x, int32_t _y, int32_t _z): x(_x), y(_y), z(_z) {}
    
    Point3 operator+(const Point3& p) const { return Point3(x+p.x, y+p.y, z+p.z); }
    Point3 operator-(const Point3& p) const { return Point3(x-p.x, y-p.y, z-p.z); }
    Point3 operator/(const int32_t i) const { return Point3(x/i, y/i, z/i); }
    
    Point3& operator += (const Point3& p) { x += p.x; y += p.y; z += p.z; return *this; }
    Point3& operator -= (const Point3& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
    
    bool operator==(Point3& p) const { return x==p.x&&y==p.y&&z==p.z; }
    bool operator!=(Point3& p) const { return x!=p.x||y!=p.y||z!=p.z; }
    
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
    
    int32_t vSize2()
    {
        return x*x+y*y+z*z;
    }
    
    int32_t vSize()
    {
        return sqrt(vSize2());
    }
};

/*
class Point
{
public:
    int32_t x,y;
    Point() {}
    Point(int32_t _x, int32_t _y): x(_x), y(_y) {}
    Point(ClipperLib::IntPoint p): x(p.X), y(p.Y) {}

    Point operator+(const Point& p) const { return Point(x+p.x, y+p.y); }
    Point operator-(const Point& p) const { return Point(x-p.x, y-p.y); }
    Point operator/(const int32_t i) const { return Point(x/i, y/i); }
    
    Point& operator += (const Point& p) { x += p.x; y += p.y; return *this; }
    Point& operator -= (const Point& p) { x -= p.x; y -= p.y; return *this; }
    
    bool operator==(Point& p) const { return x==p.x&&y==p.y; }
    bool operator!=(Point& p) const { return x!=p.x||y!=p.y; }
    
    int32_t max()
    {
        if (x > y) return x;
        return y;
    }
};*/
typedef ClipperLib::IntPoint Point;

INLINE Point operator+(const Point& p0, const Point& p1) { return Point(p0.X+p1.X, p0.Y+p1.Y); }
INLINE Point operator-(const Point& p0, const Point& p1) { return Point(p0.X-p1.X, p0.Y-p1.Y); }
INLINE Point operator/(const Point& p0, const int32_t i) { return Point(p0.X/i, p0.Y/i); }

//Point& operator += (const Point& p) { x += p.x; y += p.y; return *this; }
//Point& operator -= (const Point& p) { x -= p.x; y -= p.y; return *this; }

INLINE bool operator==(const Point& p0, const Point& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
INLINE bool operator!=(const Point& p0, const Point& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

INLINE int64_t vSize2(const Point& p0)
{
    return p0.X*p0.X+p0.Y*p0.Y;
}

INLINE bool shorterThen(const Point& p0, int32_t len)
{
    if (p0.X > len || p0.X < -len)
        return false;
    if (p0.Y > len || p0.Y < -len)
        return false;
    return vSize2(p0) <= len*len;
}

INLINE int32_t vSize(const Point& p0)
{
    return sqrt(vSize2(p0));
}

INLINE double vSizeMM(const Point& p0)
{
    double fx = double(p0.X) / 1000.0;
    double fy = double(p0.Y) / 1000.0;
    return sqrt(fx*fx+fy*fy);
}

INLINE Point normal(const Point& p0, int32_t len)
{
    int32_t _len = vSize(p0);
    return Point(p0.X * len / _len, p0.Y * len / _len);
}

INLINE Point crossZ(const Point& p0)
{
    return Point(-p0.Y, p0.X);
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
    
    void apply(Polygons& polys) const
    {
        for(unsigned int i=0; i<polys.size(); i++)
        {
            for(unsigned int j=0; j<polys[i].size(); j++)
            {
                polys[i][j] = apply(polys[i][j]);
            }
        }
    }
};

/* Axis aligned boundary box */
class AABB
{
public:
    Point min, max;
    
    AABB()
    : min(LLONG_MIN, LLONG_MIN), max(LLONG_MIN, LLONG_MIN)
    {
    }
    AABB(Polygons polys)
    : min(LLONG_MIN, LLONG_MIN), max(LLONG_MIN, LLONG_MIN)
    {
        calculate(polys);
    }
    
    void calculate(Polygons polys)
    {
        min = Point(LLONG_MAX, LLONG_MAX);
        max = Point(LLONG_MIN, LLONG_MIN);
        for(unsigned int i=0; i<polys.size(); i++)
        {
            for(unsigned int j=0; j<polys[i].size(); j++)
            {
                if (min.X > polys[i][j].X) min.X = polys[i][j].X;
                if (min.Y > polys[i][j].Y) min.Y = polys[i][j].Y;
                if (max.X < polys[i][j].X) max.X = polys[i][j].X;
                if (max.Y < polys[i][j].Y) max.Y = polys[i][j].Y;
            }
        }
    }
    
    bool hit(const AABB& other) const
    {
        if (max.X < other.min.X) return false;
        if (min.X > other.max.X) return false;
        if (max.Y < other.min.Y) return false;
        if (min.Y > other.max.Y) return false;
        return true;
    }
};

#endif//INT_POINT_H
