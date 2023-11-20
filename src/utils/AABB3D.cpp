// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/AABB3D.h"

#include <limits>

#include "utils/AABB.h"

namespace cura
{

AABB3D::AABB3D()
    : min(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max())
    , max(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min())
{
}

AABB3D::AABB3D(Point3 min, Point3 max)
    : min(min)
    , max(max)
{
}

Point3 AABB3D::getMiddle() const
{
    return (min + max) / 2;
}

AABB AABB3D::flatten() const
{
    return AABB(Point(min.x_, min.y_), Point(max.x_, max.y_));
}


bool AABB3D::hit(const AABB3D& other) const
{
    if (max.x_ < other.min.x_ || min.x_ > other.max.x_ || max.y_ < other.min.y_ || min.y_ > other.max.y_ || max.z_ < other.min.z_ || min.z_ > other.max.z_)
    {
        return false;
    }
    return true;
}

AABB3D AABB3D::include(Point3 p)
{
    min.x_ = std::min(min.x_, p.x_);
    min.y_ = std::min(min.y_, p.y_);
    min.z_ = std::min(min.z_, p.z_);
    max.x_ = std::max(max.x_, p.x_);
    max.y_ = std::max(max.y_, p.y_);
    max.z_ = std::max(max.z_, p.z_);
    return *this;
}

AABB3D AABB3D::include(const AABB3D& aabb)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min.x_ = std::min(min.x_, aabb.min.x_);
    min.y_ = std::min(min.y_, aabb.min.y_);
    min.z_ = std::min(min.z_, aabb.min.z_);
    max.x_ = std::max(max.x_, aabb.max.x_);
    max.y_ = std::max(max.y_, aabb.max.y_);
    max.z_ = std::max(max.z_, aabb.max.z_);
    return *this;
}

AABB3D AABB3D::includeZ(coord_t z)
{
    min.z_ = std::min(min.z_, z);
    max.z_ = std::max(max.z_, z);
    return *this;
}

AABB3D AABB3D::translate(Point3 offset)
{
    min += offset;
    max += offset;
    return *this;
}

AABB3D AABB3D::translate(Point offset)
{
    min += offset;
    max += offset;
    return *this;
}

AABB3D AABB3D::expand(coord_t outset)
{
    min -= Point3(outset, outset, outset);
    max += Point3(outset, outset, outset);
    if (min.x_ > max.x_ || min.y_ > max.y_ || min.z_ > max.z_)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

AABB3D AABB3D::expandXY(coord_t outset)
{
    min -= Point3(outset, outset, 0);
    max += Point3(outset, outset, 0);
    if (min.x_ > max.x_ || min.y_ > max.y_)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

} // namespace cura
