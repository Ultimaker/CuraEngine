// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/point3ll.h" //The headers we're implementing.

namespace cura
{

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

bool Point3LL::operator==(const Point3LL& p) const
{
    return x_ == p.x_ && y_ == p.y_ && z_ == p.z_;
}

bool Point3LL::operator!=(const Point3LL& p) const
{
    return x_ != p.x_ || y_ != p.y_ || z_ != p.z_;
}

} // namespace cura
