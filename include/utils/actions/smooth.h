// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SMOOTH_H
#define UTILS_VIEWS_SMOOTH_H

#include <numbers>
#include <vector>

#include <range/v3/action/action.hpp>
#include <range/v3/action/erase.hpp>
#include <range/v3/action/remove_if.hpp>
#include <range/v3/action/transform.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/iterator/concepts.hpp>
#include <range/v3/range_fwd.hpp>
#include <range/v3/utility/concepts.hpp>
#include <range/v3/view.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>

#include "utils/types/arachne.h"
#include "utils/types/geometry.h"
#include "utils/types/get.h"

namespace cura::actions
{

struct smooth_fn
{
    constexpr auto operator()(const std::integral auto max_resolution, const std::integral auto smooth_distance, const std::floating_point auto fluid_angle) const
    {
        return ranges::make_action_closure(ranges::bind_back(smooth_fn{}, max_resolution, smooth_distance, fluid_angle));
    }

    template<class Rng>
    requires ranges::forward_range<Rng> && ranges::sized_range<Rng>
//    requires ranges::forward_range<Rng> && ranges::sized_range<Rng> && ranges::erasable_range<Rng, ranges::iterator_t<Rng>, ranges::sentinel_t<Rng>> && (utils::point2d<ranges::range_value_t<Rng>> || utils::junctions<Rng>)
    auto operator()(Rng&& rng, const std::integral auto max_resolution, const std::integral auto smooth_distance, const std::floating_point auto fluid_angle) const
    {
        if (smooth_distance == 0)
        {
            return static_cast<Rng&&>(rng);
        }
        const auto size = ranges::distance(rng) - 1; // For closed Path, if open then subtract 0
        if (size < 3)
        {
            return static_cast<Rng&&>(rng);
        }

        using point_type = std::remove_cvref_t<decltype(*ranges::begin(rng))>;
        std::set<point_type*> to_remove;

        // Create a range of pointers to the points in the path, using the sliding view doesn't work because of the changing size of the path, which happens
        // when points are filtered out.
        auto windows = rng | ranges::views::cycle
                     | ranges::views::addressof
                     | ranges::views::filter([&to_remove](auto point) { return ! to_remove.contains(point); });

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try to shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        // TODO: Maybe smooth out depending on the angle between the segments?
        // TODO: Maybe create a sharp corner instead of a smooth one, based on minimizing the area to be added or removed?
        const auto max_distance_squared = max_resolution * max_resolution;
        const auto shift_smooth_distance = smooth_distance * 2;

        for (auto windows_it = ranges::begin(windows); windows_it != ranges::end(windows); ++windows_it)
        {
            if (*std::next(windows_it, 3) == ranges::front(windows))
            {
                break;
            }
            auto p0 = *windows_it;
            auto p1 = *std::next(windows_it, 1);
            auto p2 = *std::next(windows_it, 2);
            auto p3 = *std::next(windows_it, 3);

            const auto distance_squared = std::abs(dotProduct(p1, p2));
            if (distance_squared < max_distance_squared && ! withinDeviation(p0, p1, p2, p3, fluid_angle))
            {
                const auto p0p1_distance = std::hypot(std::get<"X">(*p1) - std::get<"X">(*p0), std::get<"Y">(*p1) - std::get<"Y">(*p0));
                const bool shift_p1 = p0p1_distance > shift_smooth_distance;
                spdlog::debug("p0p1_distance: {}, shift_p1: {}", p0p1_distance, shift_p1);
                if (shift_p1)
                {
                    // shift p1 towards p0 with the smooth distance
                    const auto shift_distance = smooth_distance / p0p1_distance;
                    const auto shift_distance_x = (std::get<"X">(*p1) - std::get<"X">(*p0)) * shift_distance;
                    const auto shift_distance_y = (std::get<"Y">(*p1) - std::get<"Y">(*p0)) * shift_distance;
                    if constexpr (utils::junctions<Rng>)
                    {
                        p1->p.X -= shift_distance_x;
                        p1->p.Y -= shift_distance_y;
                    }
                    else
                    {
                        p1->X -= shift_distance_x;
                        p1->Y -= shift_distance_y;
                    }
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(p1);
                }
                const auto p2p3_distance = std::hypot(std::get<"X">(*p3) - std::get<"X">(*p2), std::get<"Y">(*p3) - std::get<"Y">(*p2));
                const bool shift_p2 = p2p3_distance > shift_smooth_distance;
                spdlog::debug("p2p3_distance: {}, shift_p2: {}", p2p3_distance, shift_p2);
                if (shift_p2)
                {
                    // shift p2 towards p3 with the smooth distance
                    const auto shift_distance = smooth_distance / p2p3_distance ;
                    const auto shift_distance_x = (std::get<"X">(*p3) - std::get<"X">(*p2)) * shift_distance;
                    const auto shift_distance_y = (std::get<"Y">(*p3) - std::get<"Y">(*p2)) * shift_distance;
                    if constexpr (utils::junctions<Rng>)
                    {
                        p2->p.X += shift_distance_x;
                        p2->p.Y += shift_distance_y;
                    }
                    else
                    {
                        p2->X += shift_distance_x;
                        p2->Y += shift_distance_y;
                    }
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(p2);
                }
            }
        }

        return static_cast<Rng&&>(ranges::actions::remove_if(rng, [&to_remove](auto point){ return to_remove.contains(&point); }));
    }

private:
    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    constexpr auto dotProduct(Vector* p0, Vector* p1) const
    {
        return std::get<"X">(*p0) * std::get<"X">(*p1) + std::get<"Y">(*p0) * std::get<"Y">(*p1);
    }

    template<utils::point2d Vector>
    constexpr auto angleBetweenVectors(Vector* vec0, Vector* vec1) const -> decltype(dotProduct(vec0, vec1))
    {
        auto dot = dotProduct(vec0, vec1);
        auto vec0_mag = std::hypot(std::get<"X">(*vec0), std::get<"Y">(*vec0));
        auto vec1_mag = std::hypot(std::get<"X">(*vec1), std::get<"Y">(*vec1));
        if (vec0_mag == 0 || vec1_mag == 0)
        {
            return 90.0;
        }
        auto cos_angle = dot / (vec0_mag * vec1_mag);
        auto angle_rad = std::acos(cos_angle);
        return angle_rad * 180.0 / std::numbers::pi;
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    constexpr auto withinDeviation(Vector* p0, Vector* p1, Vector* p2, Vector* p3, const std::floating_point auto fluid_angle) const
    {
        struct Point
        {
            int64_t X;
            int64_t Y;
        };
        Point ab{ std::get<"X">(*p1) - std::get<"X">(*p0), std::get<"Y">(*p1) - std::get<"Y">(*p0) };
        Point bc{ std::get<"X">(*p2) - std::get<"X">(*p1), std::get<"Y">(*p2) - std::get<"Y">(*p1) };
        Point cd{ std::get<"X">(*p3) - std::get<"X">(*p2), std::get<"Y">(*p3) - std::get<"Y">(*p2) };
        return std::abs(angleBetweenVectors(&ab, &bc) - angleBetweenVectors(&ab, &cd)) < fluid_angle;
    }
};

inline constexpr smooth_fn smooth{};
} // namespace cura::actions

#endif // UTILS_VIEWS_SMOOTH_H
