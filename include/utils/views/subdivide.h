// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SUBDIVIDE_H
#define UTILS_VIEWS_SUBDIVIDE_H

#include <range/v3/view/join.hpp>
#include <range/v3/range/conversion.hpp>

#include "utils/types/geometry.h"

namespace cura::views
{
    namespace details
    {
        template<typename StaticFunctor>
        struct subdivide_view_fn
        {
            template<ranges::viewable_range Rng>
            /* requires concepts::segment_container<std::remove_cvref_t<Rng>>&& std::floating_point<typename Container::value_type> */
            auto operator()(Rng&& rng) const
            {
                using stops_t = decltype(std::function{StaticFunctor::stops})::result_type::value_type;
                return
                    rng |
                    ranges::view::transform
                    (
                        [](const auto& segment)
                        {
                            const auto& pa = std::get<0>(segment);
                            const auto& pb = std::get<1>(segment);
                            const auto slots_ = StaticFunctor::stops(vSize(pb - pa));
                            return
                                slots_ |
                                ranges::view::transform([&pa, &pb](const stops_t& c){ return pa * (1.0 - c) + pb * c; }) |
                                ranges::views::sliding(2) |
                                ranges::views::transform([](auto&& t){ return ranges::make_common_pair(t[0], t[1]); }) |
                                ranges::to<std::vector>();
                        }
                    ) |
                    ranges::view::join;
            }
        };

        template<typename StaticFunctor, ranges::viewable_range Rng>
        constexpr auto operator|(Rng&& rng, const subdivide_view_fn<StaticFunctor>& adaptor)
        {
            return adaptor(std::forward<Rng>(rng));
        }
    } // namespace details

    namespace subdivide_stops
    {
        struct Mid
        {
            static std::vector<double> stops(const coord_t& _)
            {
                return { 0.0, 0.5, 1.0 };
            }
        };
    }

    template<typename StaticFunctor>
    RANGES_INLINE_VARIABLE(details::subdivide_view_fn<StaticFunctor>, subdivide);

} // namespace cura::views

#endif //UTILS_VIEWS_SUBDIVIDE_H
