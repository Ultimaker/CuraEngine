//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FLOAT_POINT_H
#define FLOAT_POINT_H

#include "IntPoint.h"

#include <stdint.h>
#include <math.h>


namespace cura
{
    
/*
Floating point 3D points are used during model loading as 3D vectors.
They represent millimeters in 3D space.
*/
class FPoint3
{
public:
    float x,y,z;
    FPoint3() {}
    FPoint3(float _x, float _y, float _z): x(_x), y(_y), z(_z) {}
    FPoint3(const Point3& p): x(p.x*.001), y(p.y*.001), z(p.z*.001) {}

    FPoint3 operator+(const FPoint3& p) const { return FPoint3(x+p.x, y+p.y, z+p.z); }
    FPoint3 operator-(const FPoint3& p) const { return FPoint3(x-p.x, y-p.y, z-p.z); }
    FPoint3 operator*(const float f) const { return FPoint3(x*f, y*f, z*f); }
    FPoint3 operator/(const float f) const { return FPoint3(x/f, y/f, z/f); }

    FPoint3& operator += (const FPoint3& p) { x += p.x; y += p.y; z += p.z; return *this; }
    FPoint3& operator -= (const FPoint3& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
    FPoint3& operator *= (const float f) { x *= f; y *= f; z *= f; return *this; }

    bool operator==(FPoint3& p) const { return x==p.x&&y==p.y&&z==p.z; }
    bool operator!=(FPoint3& p) const { return x!=p.x||y!=p.y||z!=p.z; }

    float max() const
    {
        if (x > y && x > z) return x;
        if (y > z) return y;
        return z;
    }

    bool testLength(float len) const
    {
        return vSize2() <= len*len;
    }

    float vSize2() const
    {
        return x*x+y*y+z*z;
    }

    float vSize() const
    {
        return sqrt(vSize2());
    }

    inline FPoint3 normalized() const
    {
        return (*this)/vSize();
    }

    FPoint3 cross(const FPoint3& p) const
    {
        return FPoint3(
            y*p.z-z*p.y,
            z*p.x-x*p.z,
            x*p.y-y*p.x);
    }

    static FPoint3 cross(const Point3& a, const Point3& b)
    {
        return FPoint3(a).cross(FPoint3(b));
//        FPoint3(
//            a.y*b.z-a.z*b.y,
//            a.z*b.x-a.x*b.z,
//            a.x*b.y-a.y*b.x);
    }

    Point3 toPoint3()
    {
        return Point3(MM2INT(x), MM2INT(y), MM2INT(z));
    }
};


//inline FPoint3 operator+(FPoint3 lhs, const FPoint3& rhs) {
//  lhs += rhs;
//  return lhs;
//}
inline float operator*(FPoint3 lhs, const FPoint3& rhs) {
    return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
}
//inline FPoint3 operator*(FPoint3 lhs, const float f) {
//  lhs *= f;
//  return lhs;
//}



}//namespace cura
#endif//INT_POINT_H
