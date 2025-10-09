// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/AABB3D.h"

#include <limits>

#include "utils/AABB.h"

namespace cura
{

AABB3D::AABB3D()
    : min_(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max())
    , max_(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min())
{
}

AABB3D::AABB3D(Point3LL min, Point3LL max)
    : min_(min)
    , max_(max)
{
}

Point3LL AABB3D::getMiddle() const
{
    return (min_ + max_) / 2;
}

AABB AABB3D::flatten() const
{
    return AABB(Point2LL(min_.x_, min_.y_), Point2LL(max_.x_, max_.y_));
}


bool AABB3D::hit(const AABB3D& other) const
{
    if (max_.x_ < other.min_.x_ || min_.x_ > other.max_.x_ || max_.y_ < other.min_.y_ || min_.y_ > other.max_.y_ || max_.z_ < other.min_.z_ || min_.z_ > other.max_.z_)
    {
        return false;
    }
    return true;
}

AABB3D AABB3D::include(const Point3LL& p)
{
    min_.x_ = std::min(min_.x_, p.x_);
    min_.y_ = std::min(min_.y_, p.y_);
    min_.z_ = std::min(min_.z_, p.z_);
    max_.x_ = std::max(max_.x_, p.x_);
    max_.y_ = std::max(max_.y_, p.y_);
    max_.z_ = std::max(max_.z_, p.z_);
    return *this;
}

AABB3D AABB3D::include(const AABB3D& aabb)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min_.x_ = std::min(min_.x_, aabb.min_.x_);
    min_.y_ = std::min(min_.y_, aabb.min_.y_);
    min_.z_ = std::min(min_.z_, aabb.min_.z_);
    max_.x_ = std::max(max_.x_, aabb.max_.x_);
    max_.y_ = std::max(max_.y_, aabb.max_.y_);
    max_.z_ = std::max(max_.z_, aabb.max_.z_);
    return *this;
}

AABB3D AABB3D::includeZ(coord_t z)
{
    min_.z_ = std::min(min_.z_, z);
    max_.z_ = std::max(max_.z_, z);
    return *this;
}

AABB3D AABB3D::translate(const Point3LL& offset)
{
    min_ += offset;
    max_ += offset;
    return *this;
}

AABB3D AABB3D::translate(const Point2LL& offset)
{
    min_ += offset;
    max_ += offset;
    return *this;
}

AABB3D AABB3D::expand(coord_t outset)
{
    min_ -= Point3LL(outset, outset, outset);
    max_ += Point3LL(outset, outset, outset);
    if (min_.x_ > max_.x_ || min_.y_ > max_.y_ || min_.z_ > max_.z_)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

AABB3D AABB3D::expandXY(coord_t outset)
{
    min_ -= Point3LL(outset, outset, 0);
    max_ += Point3LL(outset, outset, 0);
    if (min_.x_ > max_.x_ || min_.y_ > max_.y_)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

coord_t AABB3D::spanX() const
{
    return max_.x_ - min_.x_;
}

coord_t AABB3D::spanY() const
{
    return max_.y_ - min_.y_;
}

coord_t AABB3D::spanZ() const
{
    return max_.z_ - min_.z_;
}

} // namespace cura
