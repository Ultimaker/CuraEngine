// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SMOOTH_H
#define UTILS_VIEWS_SMOOTH_H

#include <functional>
#include <numbers>
#include <set>

#include <range/v3/action/remove_if.hpp>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/iterator/concepts.hpp>
#include <range/v3/range_fwd.hpp>
#include <range/v3/view/addressof.hpp>
#include <range/v3/view/cycle.hpp>
#include <range/v3/view/filter.hpp>

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
    requires ranges::forward_range<Rng> && ranges::sized_range<Rng> && ranges::erasable_range<Rng, ranges::iterator_t<Rng>, ranges::sentinel_t<Rng>> && (utils::point2d<ranges::range_value_t<Rng>> || utils::junctions<Rng>)
    constexpr auto operator()(Rng&& rng, const std::integral auto max_resolution, const std::integral auto smooth_distance, const std::floating_point auto fluid_angle) const
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
        auto tmp = rng;
        auto ref_view = ranges::views::concat(tmp, tmp | ranges::views::take(2)) | ranges::views::addressof | ranges::to_vector;
        auto windows = ref_view | ranges::views::filter([&to_remove](auto point) { return ! to_remove.contains(point); });

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try to shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        const auto shift_smooth_distance = smooth_distance * 2;

        for (auto windows_it = ranges::begin(windows); windows_it != ranges::end(windows); ++windows_it)
        {
            auto p0 = *windows_it;
            auto p1 = *std::next(windows_it, 1);
            auto p2 = *std::next(windows_it, 2);
            auto p3 = *std::next(windows_it, 3);

            const auto p1p2_distance = std::hypot(std::get<"X">(*p2) - std::get<"X">(*p1), std::get<"Y">(*p2) - std::get<"Y">(*p1));
            if (p1p2_distance < max_resolution && ! withinDeviation(p0, p1, p2, p3, fluid_angle))
            {
                const auto p0p1_distance = std::hypot(std::get<"X">(*p1) - std::get<"X">(*p0), std::get<"Y">(*p1) - std::get<"Y">(*p0));
                const bool shift_p1 = p0p1_distance > shift_smooth_distance;
                if (shift_p1)
                {
                    shiftPointTowards(p1, p0, p0p1_distance, smooth_distance);
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(p1);
                }
                const auto p2p3_distance = std::hypot(std::get<"X">(*p3) - std::get<"X">(*p2), std::get<"Y">(*p3) - std::get<"Y">(*p2));
                const bool shift_p2 = p2p3_distance > shift_smooth_distance;
                if (shift_p2)
                {
                    shiftPointTowards(p2, p3, p2p3_distance, smooth_distance);
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(p2);
                }
            }
            if (p2 == ranges::front(windows))
            {
                break;
            }
        }

        return static_cast<Rng&&>(ranges::actions::remove_if(tmp, [&to_remove](auto point){ return to_remove.contains(&point); }));
    }

private:
    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr void shiftPointTowards(Point* point, Point* target, const std::floating_point auto p0p1_distance, const std::integral auto smooth_distance) const noexcept {
        using coord_type = std::remove_cvref_t<decltype(std::get<"X">(*point))>;
        const auto shift_distance = smooth_distance / p0p1_distance;
        const auto shift_distance_x = static_cast<coord_type>((std::get<"X">(*target) - std::get<"X">(*point)) * shift_distance);
        const auto shift_distance_y = static_cast<coord_type>((std::get<"Y">(*target) - std::get<"Y">(*point)) * shift_distance);
        if constexpr (utils::point2d<Point>)
        {
            point->X += shift_distance_x;
            point->Y += shift_distance_y;
        }
        else
        {
            point->p.X += shift_distance_x;
            point->p.Y += shift_distance_y;
        }
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    constexpr auto dotProduct(Vector* point_0, Vector* point_1) const noexcept
    {
        return std::get<"X">(*point_0) * std::get<"X">(*point_1) + std::get<"Y">(*point_0) * std::get<"Y">(*point_1);
    }

    template<utils::point2d Vector>
    constexpr auto angleBetweenVectors(Vector* vec0, Vector* vec1) const -> decltype(dotProduct(vec0, vec1))
    {
        const auto dot = dotProduct(vec0, vec1);
        const auto vec0_mag = std::hypot(std::get<"X">(*vec0), std::get<"Y">(*vec0));
        const auto vec1_mag = std::hypot(std::get<"X">(*vec1), std::get<"Y">(*vec1));
        if (vec0_mag == 0 || vec1_mag == 0)
        {
            constexpr auto perpendicular_angle = 90.0;
            return perpendicular_angle;
        }
        const auto cos_angle = dot / (vec0_mag * vec1_mag);
        const auto angle_rad = std::acos(cos_angle);
        constexpr auto rad_to_degree_factor = 180.0 / std::numbers::pi;
        return angle_rad * rad_to_degree_factor;
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
