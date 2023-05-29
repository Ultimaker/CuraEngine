// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_SIMPLIFY_H
#define UTILS_GEOMETRY_SIMPLIFY_H

#include <concepts>
#include <range/v3/view/drop.hpp>
#include <type_traits>

#include <boost/geometry/algorithms/simplify.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/operations.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

#include "utils/types/boost_tags.h"
#include "utils/types/geometry.h"
#include "utils/views/segments.h"

namespace cura::views
{
namespace details
{
struct simplify_base_fn
{
    template<ranges::viewable_range Rng>
    requires utils::closed_path<std::remove_cvref_t<Rng>> || utils::open_path<std::remove_cvref_t<Rng>> || utils::filled_path<std::remove_cvref_t<Rng>>
    constexpr auto operator()(Rng&& rng, const std::integral auto max_deviation) const
    {
        using point_t = std::remove_cvref_t<decltype(*ranges::begin(rng))>;
        boost::geometry::model::ring<point_t> simplified;
        boost::geometry::simplify(rng, simplified, max_deviation);
        return simplified | ranges::views::all;
    }

    template<ranges::viewable_range Rng>
    requires utils::ranged_path<std::remove_cvref_t<Rng>>
    constexpr auto operator()(Rng&& rng, const std::integral auto max_deviation) const
    {
        return rng | ranges::views::transform([this, max_deviation](auto&& sub_rng) { return operator()(sub_rng, max_deviation); }) | ranges::views::all;
    }
};

struct simplify_fn : simplify_base_fn
{
    using simplify_base_fn::operator();

    auto operator()(const std::integral auto max_deviation) const
    {
        return ranges::make_view_closure(ranges::bind_back(simplify_base_fn{}, max_deviation));
    }
};

} // namespace details

RANGES_INLINE_VARIABLE(details::simplify_fn, simplify);


} // namespace cura::views

#endif // UTILS_GEOMETRY_SIMPLIFY_H