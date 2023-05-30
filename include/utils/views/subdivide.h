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
        private:
            coord_t min_length_;

        public:
            subdivide_view_fn() = delete;
            subdivide_view_fn(const coord_t& min_length) : min_length_(min_length) {}

            template<ranges::viewable_range Rng> requires utils::segment_range<std::remove_cvref_t<Rng>>
            auto operator()(Rng&& rng) const
            {
                using stops_t = decltype(std::function{StaticFunctor::stops})::result_type::value_type;
                const auto& min_length = min_length_;
                return
                    rng |
                    ranges::view::transform
                    (
                        [min_length](const auto& segment)
                        {
                            const auto& pa = std::get<0>(segment);
                            const auto& pb = std::get<1>(segment);
                            const auto slots_ = StaticFunctor::stops(vSize(pb - pa), min_length);
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

            template<ranges::viewable_range Rng> requires utils::segment_range_range<std::remove_cvref_t<Rng>>
            auto operator()(Rng&& rng) const
            {
                return rng | ranges::views::transform([this](auto&& sub_rng) { return operator()(std::forward<decltype(sub_rng)>(sub_rng)); }) | ranges::views::all;
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
            static std::vector<double> stops(const coord_t& _, const coord_t& __)
            {
                return { 0.0, 0.5, 1.0 };
            }
        };

        struct Simplify0
        {
            static std::vector<double> stops(const coord_t& len, const coord_t& min_len)
            {
                if (len <= min_len)
                {
                    return { 0.0, 1.0 };
                }
                else if (len <= 2 * min_len)
                {
                    return { 0.0, 0.5, 1.0 };
                }
                const auto stop1 = static_cast<double>(len) / min_len;
                return { 0.0, stop1, 1.0 - stop1, 1.0};
            }
        };
    }

    template<typename StaticFunctor>
    auto subdivide(const coord_t& min_length)
    {
        return details::subdivide_view_fn<StaticFunctor>(min_length);
    }

} // namespace cura::views

#endif //UTILS_VIEWS_SUBDIVIDE_H
