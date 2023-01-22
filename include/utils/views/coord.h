// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VIEWS_COORD_H
#define INCLUDE_UTILS_VIEWS_COORD_H

#include <coroutine>

#include <range/v3/view/for_each.hpp>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/view.hpp>

#include "utils/Coord_t.h"
#include "utils/concepts/geometry.h"
#include <utils/views/generator.h>

namespace cura::views
{
namespace details
{

// TODO: use actual type of coordinates in point for generator template arguments

[[maybe_unused]] generator<coord_t>
coord_view_fn(const point3d_named auto& point)
{
    co_yield point.x;
    co_yield point.y;
    co_yield point.z;
}

[[maybe_unused]] generator<coord_t>
coord_view_fn(const point2d_named auto& point)
{
    co_yield point.X;
    co_yield point.Y;
}

[[maybe_unused]] generator<coord_t>
coord_view_fn(const point auto& point)
{
    if constexpr ( point_named<decltype( point )> )
    {
        for ( auto coord : coord_view_fn( point ))
        {
            co_yield coord;
        }
    }
    else
    {
        for ( auto coord : point )
        {
            co_yield coord;
        }
    }
}
} // namespace details

[[nodiscard]] constexpr auto coord_view(const point auto& point)
{
    auto coords = details::coord_view_fn( point );
    return ranges::make_view_closure( ranges::make_subrange( coords.begin(), coords.end()));
}
} // namespace cura::views

#endif //INCLUDE_UTILS_VIEWS_COORD_H
