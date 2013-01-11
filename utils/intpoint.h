#ifndef INT_POINT_H
#define INT_POINT_H

/*
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/

#include "clipper/clipper.hpp"

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
    
    bool shorterThen(int32_t len)
    {
        if (x > len || x < -len)
            return false;
        if (y > len || y < -len)
            return false;
        return vSize2() <= len*len;
    }
    
    int32_t vSize2()
    {
        return x*x+y*y;
    }

    int32_t vSize()
    {
        return sqrt(vSize2());
    }
    
    float vSizeMM()
    {
        float fx = float(x) / 1000;
        float fy = float(y) / 1000;
        return sqrtf(fx*fx+fy*fy);
    }
    
    ClipperLib::IntPoint IntPoint()
    {
        return ClipperLib::IntPoint(x, y);
    }
};

#endif//INT_POINT_H
