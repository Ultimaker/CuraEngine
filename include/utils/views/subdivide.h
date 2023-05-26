// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SUBDIVIDE_H
#define UTILS_VIEWS_SUBDIVIDE_H

#include <range/v3/view/join.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{
namespace details
{
struct subdivide_view_fn
{
    template<ranges::viewable_range Rng>
    constexpr auto operator()(Rng && rng) const
    {
        return
            rng |
            ranges::view::transform
            (
                [](const auto& segment)
                {
                    std::vector<std::pair<Point, Point>> res;

                    // TODO: Just a midpoint for now. Give a formula or at least do this more intelligently.
                    const auto mid = (segment.first + segment.second) / 2;
                    res.push_back({ segment.first, mid });
                    res.push_back({ mid, segment.second });
                    
                    return res;
                }
            ) |
            ranges::view::join;
    }
};

template<ranges::viewable_range Rng>
constexpr auto operator|(Rng&& rng, const subdivide_view_fn& adaptor)
{
    return adaptor(std::forward<Rng>(rng));
}
} // namespace details

RANGES_INLINE_VARIABLE(details::subdivide_view_fn, subdivide);

} // namespace cura::views

#endif //UTILS_VIEWS_SUBDIVIDE_H
