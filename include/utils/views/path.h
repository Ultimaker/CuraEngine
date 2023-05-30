// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_PATH_H
#define UTILS_VIEWS_PATH_H

#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>
#include <type_traits>

#include "utils/types/geometry.h"

namespace cura::views
{
namespace details
{
struct path_view_fn
{
    template<ranges::viewable_range Rng>
    requires utils::segment_range<Rng> constexpr auto operator()(Rng&& rng) const
    {
        return rng | ranges::views::transform([](auto&& sub_rng) { return std::get<0>(sub_rng); });
    }

    template<ranges::viewable_range Rng>
    requires utils::segment_range_range<std::remove_cvref_t<Rng>>
    constexpr auto operator()(Rng&& rng) const
    {
        return rng | ranges::views::transform([this](auto&& sub_rng) { return operator()(std::forward<decltype(sub_rng)>(sub_rng)); }) | ranges::views::all;
    }
};

template<ranges::viewable_range Rng>
constexpr auto operator|(Rng&& rng, const path_view_fn& adaptor)
{
    return adaptor(std::forward<Rng>(rng));
}
} // namespace details

RANGES_INLINE_VARIABLE(details::path_view_fn, path);

} // namespace cura::views

#endif // UTILS_VIEWS_PATH_H
