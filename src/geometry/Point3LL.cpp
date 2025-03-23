// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/Point3LL.h" //The headers we're implementing.

namespace cura
{

Point3LL::Point3LL(const Point2LL& point, const coord_t z)
    : x_(point.X)
    , y_(point.Y)
    , z_(z)
{
}

Point3LL Point3LL::operator+(const Point3LL& p) const
{
    return Point3LL(x_ + p.x_, y_ + p.y_, z_ + p.z_);
}

Point3LL Point3LL::operator-() const
{
    return Point3LL(-x_, -y_, -z_);
}

Point3LL Point3LL::operator-(const Point3LL& p) const
{
    return Point3LL(x_ - p.x_, y_ - p.y_, z_ - p.z_);
}

Point3LL Point3LL::operator*(const Point3LL& p) const
{
    return Point3LL(x_ * p.x_, y_ * p.y_, z_ * p.z_);
}

Point3LL Point3LL::operator/(const Point3LL& p) const
{
    return Point3LL(x_ / p.x_, y_ / p.y_, z_ / p.z_);
}

Point3LL& Point3LL::operator+=(const Point3LL& p)
{
    x_ += p.x_;
    y_ += p.y_;
    z_ += p.z_;
    return *this;
}

Point3LL& Point3LL::operator-=(const Point3LL& p)
{
    x_ -= p.x_;
    y_ -= p.y_;
    z_ -= p.z_;
    return *this;
}

Point3LL& Point3LL::operator*=(const Point3LL& p)
{
    x_ *= p.x_;
    y_ *= p.y_;
    z_ *= p.z_;
    return *this;
}

Point3LL& Point3LL::operator/=(const Point3LL& p)
{
    x_ /= p.x_;
    y_ /= p.y_;
    z_ /= p.z_;
    return *this;
}

Point2LL Point3LL::toPoint2LL() const
{
    return Point2LL(x_, y_);
}

Point3LL Point3LL::resized(coord_t length) const
{
    const coord_t actual_length = vSize();
    if (actual_length < 1)
    {
        return { length, 0, 0 };
    }
    return ((*this) * length) / actual_length;
}

} // namespace cura
