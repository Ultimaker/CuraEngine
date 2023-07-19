// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

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

AABB3D::AABB3D(Point3 min, Point3 max) : min(min), max(max)
{
}

Point3 AABB3D::getMiddle() const
{
    return (min + max) / 2;
}

AABB AABB3D::flatten() const
{
    return AABB(Point(min.x, min.y), Point(max.x, max.y));
}


bool AABB3D::hit(const AABB3D& other) const
{
    if (max.x < other.min.x || min.x > other.max.x || max.y < other.min.y || min.y > other.max.y || max.z < other.min.z || min.z > other.max.z)
    {
        return false;
    }
    return true;
}

AABB3D AABB3D::include(Point3 p)
{
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
    return *this;
}

AABB3D AABB3D::include(const AABB3D& aabb)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min.x = std::min(min.x, aabb.min.x);
    min.y = std::min(min.y, aabb.min.y);
    min.z = std::min(min.z, aabb.min.z);
    max.x = std::max(max.x, aabb.max.x);
    max.y = std::max(max.y, aabb.max.y);
    max.z = std::max(max.z, aabb.max.z);
    return *this;
}

AABB3D AABB3D::includeZ(coord_t z)
{
    min.z = std::min(min.z, z);
    max.z = std::max(max.z, z);
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
    if (min.x > max.x || min.y > max.y || min.z > max.z)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

AABB3D AABB3D::expandXY(coord_t outset)
{
    min -= Point3(outset, outset, 0);
    max += Point3(outset, outset, 0);
    if (min.x > max.x || min.y > max.y)
    { // make this AABB3D invalid
        *this = AABB3D();
    }
    return *this;
}

} // namespace cura
