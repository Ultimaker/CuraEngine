// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Point3.h" //The headers we're implementing.

namespace cura
{

Point3 Point3::operator+(const Point3& p) const
{
    return Point3(x_ + p.x_, y_ + p.y_, z_ + p.z_);
}

Point3 Point3::operator-() const
{
    return Point3(-x_, -y_, -z_);
}

Point3 Point3::operator-(const Point3& p) const
{
    return Point3(x_ - p.x_, y_ - p.y_, z_ - p.z_);
}

Point3 Point3::operator*(const Point3& p) const
{
    return Point3(x_ * p.x_, y_ * p.y_, z_ * p.z_);
}

Point3 Point3::operator/(const Point3& p) const
{
    return Point3(x_ / p.x_, y_ / p.y_, z_ / p.z_);
}

Point3& Point3::operator+=(const Point3& p)
{
    x_ += p.x_;
    y_ += p.y_;
    z_ += p.z_;
    return *this;
}

Point3& Point3::operator-=(const Point3& p)
{
    x_ -= p.x_;
    y_ -= p.y_;
    z_ -= p.z_;
    return *this;
}

Point3& Point3::operator*=(const Point3& p)
{
    x_ *= p.x_;
    y_ *= p.y_;
    z_ *= p.z_;
    return *this;
}

Point3& Point3::operator/=(const Point3& p)
{
    x_ /= p.x_;
    y_ /= p.y_;
    z_ /= p.z_;
    return *this;
}

bool Point3::operator==(const Point3& p) const
{
    return x_ == p.x_ && y_ == p.y_ && z_ == p.z_;
}

bool Point3::operator!=(const Point3& p) const
{
    return x_ != p.x_ || y_ != p.y_ || z_ != p.z_;
}

} // namespace cura
