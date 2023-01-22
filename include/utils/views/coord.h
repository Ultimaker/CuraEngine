// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VIEWS_COORD_H
#define INCLUDE_UTILS_VIEWS_COORD_H

#include <array>

#include <range/v3/view/subrange.hpp>
#include <range/v3/view/view.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{

[[nodiscard]] constexpr auto coord_view(const point_ranged auto& point)
{
    return ranges::make_view_closure( ranges::make_subrange( point.begin(), point.end()));
}

[[nodiscard]] constexpr auto coord_view(const point2d_named auto& point)
{
    std::array<decltype( point.X ), 2> point_rng { point.X, point.Y };
    return ranges::make_view_closure( ranges::make_subrange( point_rng.begin(), point_rng.end()));
}

[[nodiscard]] constexpr auto coord_view(const point3d_named auto& point)
{
    std::array<decltype( point.x ), 3> point_rng { point.x, point.y, point.z };
    return ranges::make_view_closure( ranges::make_subrange( point_rng.begin(), point_rng.end()));
}
} // namespace cura::views

#endif //INCLUDE_UTILS_VIEWS_COORD_H
