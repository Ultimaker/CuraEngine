//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT3_H
#define POINT3_H

#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <stdint.h> //For int32_t and int64_t.

#define INT2MM(n) (double(n) / 1000.0)

namespace cura
{

class Point3
{
public:
    int32_t x,y,z;
    Point3() {}
    Point3(const int32_t _x, const int32_t _y, const int32_t _z): x(_x), y(_y), z(_z) {}

    Point3 operator+(const Point3& p) const;
    Point3 operator-(const Point3& p) const;
    Point3 operator/(const int32_t i) const;
    Point3 operator*(const int32_t i) const;
    Point3 operator*(const double d) const;

    Point3& operator +=(const Point3& p);
    Point3& operator -=(const Point3& p);
    Point3& operator *=(const Point3& p);
    Point3& operator /=(const Point3& p);

    bool operator==(const Point3& p) const;
    bool operator!=(const Point3& p) const;


    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const Point3& p)
    {
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }


    int32_t max() const
    {
        if (x > y && x > z) return x;
        if (y > z) return y;
        return z;
    }

    bool testLength(int32_t len) const
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

    int64_t dot(const Point3& p) const
    {
        return x*p.x + y*p.y + z*p.z;
    }

};

/*!
 * \brief Placeholder coordinate point (3D).
 *
 * Its value is something that is rarely used.
 */
static Point3 no_point3(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min());

inline Point3 operator*(const int32_t i, const Point3& rhs);

inline Point3 operator*(const double d, const Point3& rhs);

}

#endif //POINT3_H