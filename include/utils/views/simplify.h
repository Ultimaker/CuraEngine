// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_SIMPLIFY_H
#define UTILS_GEOMETRY_SIMPLIFY_H

#include <concepts>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/functional/pipeable.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/subrange.hpp>
#include <type_traits>

#include <boost/geometry/algorithms/simplify.hpp>
#include <range/v3/all.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/operations.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>
#include <utility>

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
    requires utils::closed_path<std::remove_cvref_t<Rng>> || utils::open_path<std::remove_cvref_t<Rng>> || utils::filled_path<std::remove_cvref_t<Rng>> constexpr auto operator()(Rng&& rng, const std::integral auto max_deviation) const
    {
        return impl_(std::forward<Rng>(rng), max_deviation);
    }

    template<ranges::viewable_range Rng>
    requires utils::ranged_path<std::remove_cvref_t<Rng>> constexpr auto operator()(Rng&& rng, const std::integral auto max_deviation) const
    {
        return rng | ranges::views::transform([this, max_deviation](auto&& sub_rng) { return impl_(std::forward<decltype(sub_rng)>(sub_rng), max_deviation); }) | ranges::views::all;
    }

private:
    template<ranges::viewable_range Rng>
    requires utils::closed_path<std::remove_cvref_t<Rng>> || utils::open_path<std::remove_cvref_t<Rng>> || utils::filled_path<std::remove_cvref_t<Rng>> constexpr auto impl_(Rng&& rng, const std::integral auto max_deviation) const
    {
        using rng_t = std::remove_cvref_t<Rng>;
        rng_t simplified;
        boost::geometry::simplify(rng, simplified, max_deviation);
        return simplified;
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