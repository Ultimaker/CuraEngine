// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT3LL_H
#define GEOMETRY_POINT3LL_H

#include <cassert>
#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <type_traits> // for operations on any arithmetic number type

#include "utils/Coord_t.h"


namespace cura
{

class Point3LL
{
public:
    coord_t x_, y_, z_;
    Point3LL()
    {
    }
    Point3LL(const coord_t x, const coord_t y, const coord_t z)
        : x_(x)
        , y_(y)
        , z_(z)
    {
    }

    Point3LL operator+(const Point3LL& p) const;
    Point3LL operator-() const;
    Point3LL operator-(const Point3LL& p) const;
    Point3LL operator*(const Point3LL& p) const; //!< Element-wise multiplication. For dot product, use .dot()!
    Point3LL operator/(const Point3LL& p) const;
    template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
    Point3LL operator*(const num_t i) const
    {
        return Point3LL(std::llround(static_cast<num_t>(x_) * i), std::llround(static_cast<num_t>(y_) * i), std::llround(static_cast<num_t>(z_) * i));
    }
    template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
    Point3LL operator/(const num_t i) const
    {
        return Point3LL(x_ / i, y_ / i, z_ / i);
    }
    template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
    Point3LL operator%(const num_t i) const
    {
        return Point3LL(x_ % i, y_ % i, z_ % i);
    }

    Point3LL& operator+=(const Point3LL& p);
    Point3LL& operator-=(const Point3LL& p);
    Point3LL& operator*=(const Point3LL& p);
    Point3LL& operator/=(const Point3LL& p);
    template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
    Point3LL& operator*=(const num_t i)
    {
        x_ *= i;
        y_ *= i;
        z_ *= i;
        return *this;
    }
    template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
    Point3LL& operator/=(const num_t i)
    {
        x_ /= i;
        y_ /= i;
        z_ /= i;
        return *this;
    }

    bool operator==(const Point3LL& p) const;
    bool operator!=(const Point3LL& p) const;


    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const Point3LL& p)
    {
        return os << "(" << p.x_ << ", " << p.y_ << ", " << p.z_ << ")";
    }


    coord_t max() const
    {
        if (x_ > y_ && x_ > z_)
            return x_;
        if (y_ > z_)
            return y_;
        return z_;
    }

    bool testLength(coord_t len) const
    {
        if (x_ > len || x_ < -len)
            return false;
        if (y_ > len || y_ < -len)
            return false;
        if (z_ > len || z_ < -len)
            return false;
        return vSize2() <= len * len;
    }

    coord_t vSize2() const
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }

    coord_t vSize() const
    {
        return std::llrint(sqrt(static_cast<double>(vSize2())));
    }

    double vSizeMM() const
    {
        double fx = INT2MM(x_);
        double fy = INT2MM(y_);
        double fz = INT2MM(z_);
        return sqrt(fx * fx + fy * fy + fz * fz);
    }

    coord_t dot(const Point3LL& p) const
    {
        return x_ * p.x_ + y_ * p.y_ + z_ * p.z_;
    }

    coord_t& operator[](const size_t index)
    {
        assert(index < 3);
        switch (index)
        {
        case 0:
            return x_;
        case 1:
            return y_;
        default:
            return z_;
        }
    }
    const coord_t& operator[](const size_t index) const
    {
        return const_cast<Point3LL*>(this)->operator[](index);
    }
};

/*!
 * \brief Placeholder coordinate point (3D).
 *
 * Its value is something that is rarely used.
 */
static Point3LL no_point3(std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min());

template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
inline Point3LL operator*(const num_t i, const Point3LL& rhs)
{
    return rhs * i;
}

} // namespace cura


namespace std
{
template<>
struct hash<cura::Point3LL>
{
    size_t operator()(const cura::Point3LL& pp) const
    {
        static int prime = 31;
        int result = 89;
        result = static_cast<int>(result * prime + pp.x_);
        result = static_cast<int>(result * prime + pp.y_);
        result = static_cast<int>(result * prime + pp.z_);
        return static_cast<size_t>(result);
    }
};
} // namespace std


#endif // GEOMETRY_POINT3LL_H
