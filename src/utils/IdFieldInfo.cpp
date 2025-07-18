// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/IdFieldInfo.h"

#include <algorithm>
#include <array>
#include <tuple>

using namespace cura;

coord_t IdFieldInfo::getAxisValue(const Axis ax, const Point3LL& pt)
{
    switch (ax)
    {
    case Axis::X:
        return pt.x_;
    case Axis::Y:
        return pt.y_;
    case Axis::Z:
        return pt.z_;
    default:
        return 0;
    }
}

std::optional<IdFieldInfo> IdFieldInfo::fromAabb3d(const AABB3D& aabb)
{
    if (! aabb.exists())
    {
        return std::nullopt;
    }

    typedef std::tuple<IdFieldInfo::Axis, coord_t, coord_t> axis_span_t;
    std::array<axis_span_t, 3> dif_per_axis = { std::make_tuple(IdFieldInfo::Axis::X, aabb.min_.x_, aabb.max_.x_),
                                                std::make_tuple(IdFieldInfo::Axis::Y, aabb.min_.y_, aabb.max_.y_),
                                                std::make_tuple(IdFieldInfo::Axis::Z, aabb.min_.z_, aabb.max_.z_) };
    std::stable_sort(
        dif_per_axis.begin(),
        dif_per_axis.end(),
        [](const axis_span_t& a, const axis_span_t& b)
        {
            return std::llabs(std::get<2>(a) - std::get<1>(a)) > std::llabs(std::get<2>(b) - std::get<1>(b));
        });

    return std::make_optional(IdFieldInfo{
        .primary_axis_ = std::get<0>(dif_per_axis[0]),
        .secondary_axis_ = std::get<0>(dif_per_axis[1]),
        .normal_ = std::get<0>(dif_per_axis[2]),
        .projection_field_ = AABB(Point2LL(std::get<1>(dif_per_axis[0]), std::get<1>(dif_per_axis[1])), Point2LL(std::get<2>(dif_per_axis[0]), std::get<2>(dif_per_axis[1]))) });
}

float mirrorNegative(const float in)
{
    return in < 0.0f ? 1.0 + in : in;
}

Point2F IdFieldInfo::worldPointToLabelUv(const Point3LL& pt) const
{
    return Point2F(
        1.0f - mirrorNegative(static_cast<float>(getAxisValue(primary_axis_, pt) - projection_field_.min_.X) / (projection_field_.max_.X - projection_field_.min_.X)),
        1.0f - mirrorNegative(static_cast<float>(getAxisValue(secondary_axis_, pt) - projection_field_.min_.Y) / (projection_field_.max_.Y - projection_field_.min_.Y)));
}
