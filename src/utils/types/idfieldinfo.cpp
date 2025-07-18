#include "utils/types/idfieldinfo.h"

#include <algorithm>
#include <array>
#include <tuple>

using namespace cura;

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
        .projection_field_ = AABB(
            Point2LL(std::get<1>(dif_per_axis[0]), std::get<1>(dif_per_axis[1])),
            Point2LL(std::get<2>(dif_per_axis[0]), std::get<2>(dif_per_axis[1])))
        });
}
