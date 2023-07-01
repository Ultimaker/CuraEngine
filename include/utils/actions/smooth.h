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
#include <range/v3/iterator/operations.hpp>
#include <range/v3/range_fwd.hpp>
#include <range/v3/view/addressof.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/cycle.hpp>
#include <range/v3/view/filter.hpp>

#include "utils/types/arachne.h"
#include "utils/types/generic.h"
#include "utils/types/geometry.h"
#include "utils/types/get.h"

namespace cura::actions
{

struct smooth_fn
{
    constexpr auto operator()(const std::integral auto max_resolution, const std::floating_point auto fluid_angle) const
    {
        return ranges::make_action_closure(ranges::bind_back(smooth_fn{}, max_resolution, fluid_angle));
    }

    template<class Rng>
    requires ranges::forward_range<Rng>&& ranges::sized_range<Rng>&& ranges::erasable_range<Rng, ranges::iterator_t<Rng>, ranges::sentinel_t<Rng>> && (utils::point2d<ranges::range_value_t<Rng>> || utils::junctions<Rng>)
    constexpr auto operator()(Rng&& rng, const std::integral auto max_resolution, const std::floating_point auto fluid_angle) const
    {
        const auto size = ranges::distance(rng) - 1; // TODO: implement for open paths! The value `-1` is for closed Paths, if open then subtract `0`
        if (size < 3)
        {
            return static_cast<Rng&&>(rng);
        }

        using point_type = std::remove_cvref_t<decltype(*ranges::begin(rng))>;
        std::set<point_type*> to_remove; // Set of points that are marked for removal

        auto tmp = rng; // We don't want to shift the points of the ingoing range, therefor we create a temporary copy
        auto ref_view = ranges::views::concat(tmp, tmp) | ranges::views::addressof; // Concate twice to make sure we have enough points to shift the window, even after filtering (note: the cylce view doesn't have and end, which makes it harder to determine when to stop)
        auto windows = ref_view | ranges::views::filter([&to_remove](auto point) { return ! to_remove.contains(point); });  // Filter out the points that are marked for removal

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        for (auto windows_it = ranges::begin(windows); windows_it != ranges::end(windows); ++windows_it)
        {
            auto A = *windows_it;
            auto B = *std::next(windows_it, 1);
            if (B == ranges::front(windows) || B == ranges::back(windows) || ranges::distance(windows_it, ranges::end(windows)) < 3)
            {
                break;
            }
            auto C = *std::next(windows_it, 2);
            auto D = *std::next(windows_it, 3);

            const auto [AB_magnitude, BC_magnitude, CD_magnitude] = computeMagnitudes(A, B, C, D);
            if (! isWithinDeviations(A, B, C, D, fluid_angle, max_resolution, AB_magnitude, BC_magnitude, CD_magnitude))
            {
                if (AB_magnitude > max_resolution)
                {
                    shiftPointTowards(B, A, AB_magnitude, max_resolution / 2);
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(B);
                }
                if (CD_magnitude > max_resolution)
                {
                    shiftPointTowards(C, D, CD_magnitude, max_resolution / 2);
                }
                else if (size - to_remove.size() > 2) // Only remove if there are more than 2 points left for open-paths, or 3 for closed
                {
                    to_remove.insert(C);
                }
            }
        }

        return static_cast<Rng&&>(ranges::actions::remove_if(tmp, [&to_remove](auto point) { return to_remove.contains(&point); }));
    }

private:
    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr auto computeMagnitudes(Point* A, Point* B, Point* C, Point* D) const noexcept
    {
        const auto AB_magnitude = std::hypot(std::get<"X">(*B) - std::get<"X">(*A), std::get<"Y">(*B) - std::get<"Y">(*A));
        const auto BC_magnitude = std::hypot(std::get<"X">(*C) - std::get<"X">(*B), std::get<"Y">(*C) - std::get<"Y">(*B));
        const auto CD_magnitude = std::hypot(std::get<"X">(*D) - std::get<"X">(*C), std::get<"Y">(*D) - std::get<"Y">(*C));

        return std::make_tuple(AB_magnitude, BC_magnitude, CD_magnitude);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr auto cosAngle(Point* A, Point* B, Point* C, const std::floating_point auto AB_magnitude, const std::floating_point auto BC_magnitude) const noexcept
    {
        if (AB_magnitude == 0.0 || BC_magnitude == 0.0)
        {
            return 0.0;
        }
        auto AB = std::make_tuple(std::get<"X">(*B) - std::get<"X">(*A), std::get<"Y">(*B) - std::get<"Y">(*A));
        auto BC = std::make_tuple(std::get<"X">(*C) - std::get<"X">(*B), std::get<"Y">(*C) - std::get<"Y">(*B));

        const auto dot = dotProduct(&AB, &BC);
        return dot / (AB_magnitude * BC_magnitude);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr void shiftPointTowards(Point* point, Point* target, const std::floating_point auto p0p1_distance, const std::integral auto smooth_distance) const noexcept
    {
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
        requires utils::point2d<Vector> || utils::junction<Vector> constexpr auto dotProduct(Vector* point_0, Vector* point_1) const noexcept
    {
        return std::get<"X">(*point_0) * std::get<"X">(*point_1) + std::get<"Y">(*point_0) * std::get<"Y">(*point_1);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr auto isWithinDeviations(Point* A, Point* B, Point* C, Point* D, const std::floating_point auto fluid_angle, const std::integral auto max_resolution,
                                          const std::floating_point auto AB_magnitude, const std::floating_point auto BC_magnitude, const std::floating_point auto CD_magnitude) const noexcept
    {
        if (BC_magnitude > max_resolution)
        {
            return true;
        }
        const double cos_A = cosAngle(A, B, C, AB_magnitude, BC_magnitude);
        const double cos_B = cosAngle(A, B, D, AB_magnitude, CD_magnitude);
        // This comparison might be slightly off because the cosine function is not linear, especially for larger angles. The range of the cosine function is from -1 to 1 for the
        // input range from -pi to pi, so for small angles, the difference in the cosine of the angles can be approximately equal to the difference in the angles
        // (measured in radians). In order words, the comparison std::abs(cosAngle(A, B, C) - cosAngle(A, B, D)) < std::cos(fluid_angle) is an approximation that is most accurate
        // for small fluid angles. For fluid angles near or exceeding pi/2 radians (90 degrees), the right-hand side of the inequality can even become negative, making the
        // condition always false. Since the fluid angle is usually small, this is not a problem in practice.
        const auto abs_cos_angle = std::abs(cos_A - cos_B);
        return abs_cos_angle < std::cos(fluid_angle);
    }
};

inline constexpr smooth_fn smooth{};
} // namespace cura::actions

#endif // UTILS_VIEWS_SMOOTH_H
