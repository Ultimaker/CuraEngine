// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SUBDIVIDE_H
#define UTILS_VIEWS_SUBDIVIDE_H

#include <range/v3/view/join.hpp>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/view/view.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{
namespace details
{
struct subdivide_view_base_fn
{
    template<ranges::viewable_range Rng, class Container>
    requires /*concepts::segment_container<std::remove_cvref_t<Rng>>&&*/ std::floating_point<typename Container::value_type>
    auto operator()(Rng&& rng, const std::function<Container(const coord_t&)>& get_stops_func_) const
    {
        using stops_t = typename Container::value_type;
        return
            rng |
            ranges::view::transform
            (
                [&get_stops_func](const auto& segment)
                {
                    const auto pa = std::get<0>(segment);
                    const auto pb = std::get<1>(segment);
                    const auto stops = get_stops_func(vSize(pb - pa)); // Call now, otherwise the function will be deleted.
                    return
                        stops |
                        ranges::view::transform([&pa, &pb](const stops_t& c) { return pa * (1.0 - c) + pb * c; }) |
                        ranges::views::sliding(2) |
                        ranges::views::transform([&](auto&& t) { return ranges::make_common_pair(t[0], t[1]); });
                }
            ) |
            ranges::view::join;
    }
};

struct subdivide_view_fn : subdivide_view_base_fn
{
    using subdivide_view_base_fn::operator();

    template<class Container>
    requires std::floating_point<typename Container::value_type>
    constexpr auto operator()(const std::function<Container(const coord_t&)>& get_stops_func) const
    {
        return ranges::make_view_closure(ranges::bind_back(subdivide_view_base_fn{}, get_stops_func));
    }
};

template<ranges::viewable_range Rng>
constexpr auto operator|(Rng&& rng, const subdivide_view_fn& adaptor)
{
    return adaptor(std::forward<Rng>(rng));
}
} // namespace details

namespace subdivide_functions
{
    std::function<std::vector<double>(const coord_t&)> getMidStops
    {
        [](const coord_t& _) -> std::vector<double>
        {
            return { 0.0, 0.5 };
        }
    };
}

RANGES_INLINE_VARIABLE(details::subdivide_view_fn, subdivide);

} // namespace cura::views

#endif //UTILS_VIEWS_SUBDIVIDE_H
