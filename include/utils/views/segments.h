// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SEGMENTS_H
#define UTILS_VIEWS_SEGMENTS_H

#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/sliding.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{
namespace details
{
struct segment_view_fn
{
    template<ranges::viewable_range Rng>
    requires is_closed_container_v<std::remove_cvref_t<Rng>> constexpr auto operator()(Rng&& rng) const
    {
        return ranges::views::concat(rng, ranges::views::single(rng.front())) | ranges::views::sliding(2) | ranges::views::transform([](auto&& t) { return ranges::make_common_pair(t[0], t[1]); });
    }

    template<ranges::viewable_range Rng>
    requires is_open_container_v<std::remove_cvref_t<Rng>> constexpr auto operator()(Rng&& rng) const
    {
        return ranges::views::sliding(rng, 2) | ranges::views::transform([](auto&& t) { return ranges::make_common_pair(t[0], t[1]); });
    }
};

template<ranges::viewable_range Rng>
constexpr auto operator|(Rng&& rng, const segment_view_fn& adaptor)
{
    return adaptor(std::forward<Rng>(rng));
}
} // namespace details

RANGES_INLINE_VARIABLE(details::segment_view_fn, segments);

} // namespace cura::views


#endif // UTILS_VIEWS_SEGMENTS_H