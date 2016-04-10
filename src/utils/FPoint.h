/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_F_POINT_H
#define UTILS_F_POINT_H

#include <cmath> // sqrt
#include <iostream> // auto-serialization / auto-toString() '<<'

namespace cura
{
/*!
 * 2D coordinates represented by floats
 */
class FPoint
{
public:
    float x, y; //!< Coordinates 
    FPoint() //!< non-initializing constructor
    {}
    FPoint(float x, float y) //!< constructor
    : x(x)
    , y(y)
    {}

    FPoint operator+(const FPoint& p) const { return FPoint(x+p.x, y+p.y); }
    FPoint operator-(const FPoint& p) const { return FPoint(x-p.x, y-p.y); }
    FPoint operator/(const float i) const { return FPoint(x/i, y/i); }
    FPoint operator*(const float i) const { return FPoint(x*i, y*i); }
    FPoint operator*(const double d) const { return FPoint(d*x, d*y); }

    FPoint& operator += (const FPoint& p) { x += p.x; y += p.y; return *this; }
    FPoint& operator -= (const FPoint& p) { x -= p.x; y -= p.y; return *this; }

    bool operator==(const FPoint& p) const { return x == p.x && y == p.y; }
    bool operator!=(const FPoint& p) const { return x != p.x || y != p.y; }

    /*!
     * output to string stream in standard format
     */
    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const FPoint& p)
    {
        return os << "(" << p.x << ", " << p.y << ")";
    }

    /*!
     * squared vector size
     */
    float vSize2() const
    {
        return x * x + y * y;
    }

    /*!
     * vector size
     */
    float vSize() const
    {
        return sqrt(vSize2());
    }

    /*!
     * dot product
     */
    float dot(const FPoint& p) const
    {
        return x * p.x + y * p.y;
    }

};

} // namespace cura

#endif // UTILS_F_POINT_H