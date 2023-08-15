// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SMOOTH_H
#define UTILS_VIEWS_SMOOTH_H

#include "utils/types/arachne.h"
#include "utils/types/generic.h"
#include "utils/types/geometry.h"
#include "utils/types/get.h"

#include <range/v3/action/remove_if.hpp>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/iterator/concepts.hpp>
#include <range/v3/iterator/operations.hpp>
#include <range/v3/range_fwd.hpp>
#include <range/v3/view/addressof.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/take.hpp>

#include <functional>
#include <numbers>
#include <set>

namespace cura::actions
{

struct smooth_fn
{
    constexpr auto operator()(const utils::integral auto max_resolution, const utils::floating_point auto fluid_angle) const
    {
        return ranges::make_action_closure(ranges::bind_back(smooth_fn{}, max_resolution, fluid_angle));
    }

    template<class Rng>
    requires ranges::forward_range<Rng> && ranges::sized_range<Rng> && ranges::erasable_range<Rng, ranges::iterator_t<Rng>, ranges::sentinel_t<Rng>> &&(
        utils::point2d<ranges::range_value_t<Rng>> || utils::junctions<Rng>)constexpr auto
        operator()(Rng&& rng, const utils::integral auto max_resolution, const utils::floating_point auto fluid_angle) const
    {
        const auto size = ranges::distance(rng) - 1;
        if (size < 4)
        {
            return static_cast<Rng&&>(rng);
        }

        using coord_type = std::remove_cvref_t<decltype(std::get<"X">(*ranges::begin(rng)))>;
        const auto allowed_deviation = static_cast<coord_type>(max_resolution * 2 / 3); // The allowed deviation from the original path
        const auto smooth_distance = static_cast<coord_type>(max_resolution / 2); // The distance over which the path is smoothed

        auto tmp = rng; // We don't want to shift the points of the in-going range, therefore we create a temporary copy
        auto windows = ranges::views::concat(ranges::views::single(ranges::back(tmp)), ranges::views::concat(tmp, tmp | ranges::views::take(4))) | ranges::views::addressof;

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        for (auto windows_it = ranges::begin(windows); ranges::distance(windows_it, ranges::end(windows)) > 2; ++windows_it)
        {
            auto A = *windows_it;
            auto B = *std::next(windows_it, 1);
            auto C = *std::next(windows_it, 2);
            auto D = *std::next(windows_it, 3);

            const auto [AB_magnitude, BC_magnitude, CD_magnitude] = computeMagnitudes(A, B, C, D);
            if (! isWithinAllowedDeviations(A, B, C, D, fluid_angle, max_resolution, AB_magnitude, BC_magnitude, CD_magnitude))
            {
                if (AB_magnitude > allowed_deviation)
                {
                    shiftPointTowards(B, A, AB_magnitude, smooth_distance);
                }
                if (CD_magnitude > allowed_deviation)
                {
                    shiftPointTowards(C, D, CD_magnitude, smooth_distance);
                }
            }
        }

        return tmp;
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
    constexpr auto cosAngle(Point* A, Point* B, Point* C, const utils::floating_point auto AB_magnitude, const utils::floating_point auto BC_magnitude) const noexcept
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
    constexpr void shiftPointTowards(Point* point, Point* target, const utils::floating_point auto p0p1_distance, const utils::integral auto smooth_distance) const noexcept
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
    requires utils::point2d<Vector> || utils::junction<Vector>
    constexpr auto dotProduct(Vector* point_0, Vector* point_1) const noexcept
    {
        return std::get<"X">(*point_0) * std::get<"X">(*point_1) + std::get<"Y">(*point_0) * std::get<"Y">(*point_1);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr auto isWithinAllowedDeviations(
        Point* A,
        Point* B,
        Point* C,
        Point* D,
        const utils::floating_point auto fluid_angle,
        const utils::integral auto max_resolution,
        const utils::floating_point auto AB_magnitude,
        const utils::floating_point auto BC_magnitude,
        const utils::floating_point auto CD_magnitude) const noexcept
    {
        if (BC_magnitude > max_resolution / 10) // TODO: make dedicated front-end setting for this
        {
            return true;
        }
        const double cos_A = std::acos(cosAngle(A, B, C, AB_magnitude, BC_magnitude));
        const double cos_B = std::acos(cosAngle(A, B, D, AB_magnitude, CD_magnitude));
        const auto abs_angle = std::abs(cos_A - cos_B);
        return abs_angle < fluid_angle;
    }
};

inline constexpr smooth_fn smooth{};
} // namespace cura::actions

#endif // UTILS_VIEWS_SMOOTH_H
