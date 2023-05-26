// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_SIMPLIFY_H
#define UTILS_GEOMETRY_SIMPLIFY_H

#include <concepts>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/operations.hpp>
#include <range/v3/range_concepts.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/view.hpp>

#include "utils/Coord_t.h"
#include "utils/concepts/geometry.h"


namespace cura::views
{
namespace details
{
struct simplify_base_fn
{
    template<concepts::is_closed_point_container Rng>
    constexpr auto operator()(Rng&& rng, const std::integral auto max_deviation) const
    {
        return rng | ranges::views::all;
    }
};

struct simplify_fn : simplify_base_fn
{
    using simplify_base_fn::operator();

    constexpr auto operator()(const std::integral auto max_deviation) const
    {
        return ranges::make_view_closure(ranges::bind_back(simplify_base_fn{}, max_deviation));
    }
};

} // namespace details

RANGES_INLINE_VARIABLE(details::simplify_fn, simplify);


} // namespace cura::views

#endif // UTILS_GEOMETRY_SIMPLIFY_H