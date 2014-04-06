/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef FLOAT_POINT_H
#define FLOAT_POINT_H

/*
Floating point 3D points are used during model loading as 3D vectors.
They represent millimeters in 3D space.
*/

#include "intpoint.h"

#include <stdint.h>
#include <math.h>

class FPoint3
{
public:
    double x,y,z;
    FPoint3() {}
    FPoint3(double _x, double _y, double _z): x(_x), y(_y), z(_z) {}
    
    FPoint3 operator+(const FPoint3& p) const { return FPoint3(x+p.x, y+p.y, z+p.z); }
    FPoint3 operator-(const FPoint3& p) const { return FPoint3(x-p.x, y-p.y, z-p.z); }
    FPoint3 operator*(const double f) const { return FPoint3(x*f, y*f, z*f); }
    FPoint3 operator/(const double f) const { return FPoint3(x/f, y/f, z/f); }
    
    FPoint3& operator += (const FPoint3& p) { x += p.x; y += p.y; z += p.z; return *this; }
    FPoint3& operator -= (const FPoint3& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
    
    bool operator==(FPoint3& p) const { return x==p.x&&y==p.y&&z==p.z; }
    bool operator!=(FPoint3& p) const { return x!=p.x||y!=p.y||z!=p.z; }
    
    double max()
    {
        if (x > y && x > z) return x;
        if (y > z) return y;
        return z;
    }
    
    bool testLength(double len)
    {
        return vSize2() <= len*len;
    }
    
    double vSize2()
    {
        return x*x+y*y+z*z;
    }
    
    double vSize()
    {
        return sqrt(vSize2());
    }
};

class FMatrix3x3
{
public:
    double m[3][3];
    
    FMatrix3x3()
    {
        m[0][0] = 1.0;
        m[1][0] = 0.0;
        m[2][0] = 0.0;
        m[0][1] = 0.0;
        m[1][1] = 1.0;
        m[2][1] = 0.0;
        m[0][2] = 0.0;
        m[1][2] = 0.0;
        m[2][2] = 1.0;
    }
    
    Point3 apply(FPoint3 p)
    {
        return Point3(
            MM2INT(p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0]),
            MM2INT(p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1]),
            MM2INT(p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2]));
    }
};

#endif//INT_POINT_H
