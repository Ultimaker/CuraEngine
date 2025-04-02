// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT3LL_H
#define GEOMETRY_POINT3LL_H

#include <cassert>
#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <type_traits> // for operations on any arithmetic number type

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"
#include "utils/types/generic.h"


namespace cura
{

class Point3LL
{
public:
    coord_t x_{ 0 };
    coord_t y_{ 0 };
    coord_t z_{ 0 };

    Point3LL() = default;

    Point3LL(const coord_t x, const coord_t y, const coord_t z)
        : x_(x)
        , y_(y)
        , z_(z)
    {
    }

    Point3LL(Point3LL&& point) = default;
    Point3LL(const Point3LL& point) = default;
    Point3LL(const Point2LL& point);

    Point3LL& operator=(const Point3LL& point) = default;
    Point3LL& operator=(Point3LL&& point) = default;

    virtual ~Point3LL() = default;

    Point3LL operator+(const Point3LL& p) const;
    Point3LL operator-() const;
    Point3LL operator-(const Point3LL& p) const;
    Point3LL operator*(const Point3LL& p) const; //!< Element-wise multiplication. For dot product, use .dot()!
    Point3LL operator/(const Point3LL& p) const;

    template<utils::numeric T>
    Point3LL operator*(const T& i) const
    {
        return { std::llround(static_cast<T>(x_) * i), std::llround(static_cast<T>(y_) * i), std::llround(static_cast<T>(z_) * i) };
    }

    template<utils::numeric T>
    Point3LL operator/(const T& i) const
    {
        return { x_ / i, y_ / i, z_ / i };
    }

    template<utils::numeric T>
    Point3LL operator%(const T& i) const
    {
        return { x_ % i, y_ % i, z_ % i };
    }

    Point3LL& operator+=(const Point3LL& p);
    Point3LL& operator-=(const Point3LL& p);
    Point3LL& operator*=(const Point3LL& p);
    Point3LL& operator/=(const Point3LL& p);

    template<utils::numeric T>
    Point3LL& operator*=(const T i)
    {
        x_ *= i;
        y_ *= i;
        z_ *= i;
        return *this;
    }

    template<utils::numeric T>
    Point3LL& operator/=(const T i)
    {
        x_ /= i;
        y_ /= i;
        z_ /= i;
        return *this;
    }

    auto operator<=>(const Point3LL&) const = default;

    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const Point3LL& p)
    {
        return os << "(" << p.x_ << ", " << p.y_ << ", " << p.z_ << ")";
    }

    [[nodiscard]] coord_t max() const
    {
        if (x_ > y_ && x_ > z_)
        {
            return x_;
        }
        if (y_ > z_)
        {
            return y_;
        }
        return z_;
    }

    [[nodiscard]] bool testLength(coord_t len) const
    {
        if (x_ > len || x_ < -len)
        {
            return false;
        }
        if (y_ > len || y_ < -len)
        {
            return false;
        }
        if (z_ > len || z_ < -len)
        {
            return false;
        }
        return vSize2() <= len * len;
    }

    [[nodiscard]] coord_t vSize2() const
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }

    [[nodiscard]] double vSize2f()
    {
        return static_cast<double>(x_) * static_cast<double>(x_) + static_cast<double>(y_) * static_cast<double>(y_) + static_cast<double>(z_) * static_cast<double>(z_);
    }

    [[nodiscard]] coord_t vSize() const
    {
        return std::llrint(sqrt(static_cast<double>(vSize2())));
    }

    [[nodiscard]] double vSizeMM() const
    {
        double fx = INT2MM(x_);
        double fy = INT2MM(y_);
        double fz = INT2MM(z_);
        return sqrt(fx * fx + fy * fy + fz * fz);
    }

    [[nodiscard]] coord_t dot(const Point3LL& p) const
    {
        return x_ * p.x_ + y_ * p.y_ + z_ * p.z_;
    }

    [[nodiscard]] Point2LL toPoint2LL() const;

    [[nodiscard]] Point3LL resized(coord_t length) const;

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

template<utils::numeric T>
inline Point3LL operator*(const T i, const Point3LL& rhs)
{
    return rhs * i;
}

inline Point3LL lerp(const Point3LL& a, const Point3LL& b, const double t)
{
    return Point3LL(cura::lerp(a.x_, b.x_, t), cura::lerp(a.y_, b.y_, t), cura::lerp(a.z_, b.z_, t));
}

} // namespace cura


namespace std
{
template<>
struct hash<cura::Point3LL>
{
    size_t operator()(const cura::Point3LL& pp) const noexcept
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
