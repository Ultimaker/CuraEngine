// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SMOOTH_H
#define UTILS_VIEWS_SMOOTH_H

#include "settings/Settings.h"
#include "settings/types/Angle.h"
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
#include <spdlog/spdlog.h>

#include <functional>
#include <numbers>
#include <set>

namespace cura
{
class SmoothTest_TestSmooth_Test;
}

namespace cura::actions
{

struct smooth_fn
{
    friend class cura::SmoothTest_TestSmooth_Test;

    const auto operator()(const Settings& settings) const
    {
        const auto fluid_motion_shift_distance = settings.get<coord_t>("meshfix_fluid_motion_shift_distance");
        const auto fluid_motion_small_distance = settings.get<coord_t>("meshfix_fluid_motion_small_distance");
        const auto fluid_motion_angle = settings.get<AngleRadians>("meshfix_fluid_motion_angle").value;
        return ranges::make_action_closure(ranges::bind_back(smooth_fn{}, fluid_motion_shift_distance, fluid_motion_small_distance, fluid_motion_angle));
    }

    constexpr auto operator()(
        const utils::integral auto fluid_motion_shift_distance,
        const utils::integral auto fluid_motion_small_distance,
        const utils::floating_point auto fluid_motion_angle) const
    {
        return ranges::make_action_closure(ranges::bind_back(smooth_fn{}, fluid_motion_shift_distance, fluid_motion_small_distance, fluid_motion_angle));
    }

    template<class Rng>
    requires ranges::forward_range<Rng> && ranges::sized_range<Rng> && ranges::erasable_range<Rng, ranges::iterator_t<Rng>, ranges::sentinel_t<Rng>> &&(
        utils::point2d<ranges::range_value_t<Rng>> || utils::junctions<Rng>)constexpr auto
        operator()(
            Rng&& rng,
            const utils::integral auto fluid_motion_shift_distance,
            const utils::integral auto fluid_motion_small_distance,
            const utils::floating_point auto fluid_motion_angle) const
    {
        const auto fluid_motion_shift_distance3 = 3 * fluid_motion_shift_distance;

        const auto size = ranges::distance(rng) - 1;
        if (size < 4)
        {
            return static_cast<Rng&&>(rng);
        }

        auto tmp = rng; // We don't want to shift the points of the in-going range, therefore we create a temporary copy
        auto windows = ranges::views::concat(ranges::views::single(ranges::back(tmp)), ranges::views::concat(tmp, tmp | ranges::views::take(4))) | ranges::views::addressof;

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        for (auto windows_it = ranges::begin(windows); ranges::distance(windows_it, ranges::end(windows)) > 2; ++windows_it)
        {
            const auto A = *windows_it;
            const auto B = *std::next(windows_it, 1);
            const auto C = *std::next(windows_it, 2);
            const auto D = *std::next(windows_it, 3);

            if (dist(*A, *B) < fluid_motion_shift_distance3 || dist(*B, *C) > fluid_motion_small_distance || dist(*C, *D) < fluid_motion_shift_distance3)
            {
                continue;
            }

            const auto cos_fluid_motion_angle = std::cos(fluid_motion_angle);
            if (! isSmooth(*A, *B, *C, *D, cos_fluid_motion_angle))
            {
                *A = shiftPointTowards(*B, *A, fluid_motion_shift_distance);
                *D = shiftPointTowards(*C, *D, fluid_motion_shift_distance);
            }
        }

        return tmp;
    }

private:
    /*
     * cosine of the angle between the vectors AB and BC
     *
     * B---------C
     * | \
     * |  angle
     * |
     * |
     * A
     *
     */
    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    auto cosAngle(Point& A, Point& B, Point& C) const noexcept
    {
        return cosAngle(A, B, C, dist(A, B), dist(B, C));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    auto cosAngle(Point& A, Point& B, Point& C, const utils::floating_point auto AB_magnitude, const utils::floating_point auto BC_magnitude) const noexcept
    {
        return cosAngle(A, B, B, C, AB_magnitude, BC_magnitude);
    }

    /*
     * cosine of the angle between the vectors AB and CD
     *
     * A   C
     * |    \
     * |     \
     * B      D
     *
     * The angle will be calculated by shifting points A and C towards the origin,
     * and then calculating the angle between the vectors AB and CD.
     *
     * A,C
     * | \
     * |  \
     * B   D
     *
     */
    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    auto cosAngle(Point& A, Point& B, Point& C, Point& D) const noexcept
    {
        return cosAngle(A, B, C, D, dist(A, B), dist(C, D));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    auto cosAngle(Point& A, Point& B, Point& C, Point& D, const utils::floating_point auto AB_magnitude, const utils::floating_point auto BC_magnitude) const noexcept
    {
        Point VectorA = { std::get<"X">(B) - std::get<"X">(A), std::get<"Y">(B) - std::get<"Y">(A) };
        Point VectorB = { std::get<"X">(D) - std::get<"X">(C), std::get<"Y">(D) - std::get<"Y">(C) };

        return cosAngle(VectorA, VectorB, AB_magnitude, BC_magnitude);
    }

    /*
     * cosine of the angle between the vectors A and B
     *
     * O   (origin)
     * | \
     * |  \
     * A   B
     *
     */
    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    auto cosAngle(Vector& A, Vector& B) const noexcept
    {
        return cosAngle<Point>(A, B, magnitude(A), magnitude(B));
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    auto cosAngle(Vector& A, Vector& B, const utils::floating_point auto A_magnitude, const utils::floating_point auto B_magnitude) const noexcept
    {
        if (A_magnitude <= FLT_EPSILON || B_magnitude <= FLT_EPSILON)
        {
            return static_cast<decltype(A_magnitude * B_magnitude)>(0.0);
        }
        return static_cast<decltype(A_magnitude * B_magnitude)>(dotProduct(A, B)) / (A_magnitude * B_magnitude);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr Point shiftPointTowards(Point& p0, Point& p1, const utils::numeric auto move_distance) const noexcept
    {
        return shiftPointTowards(p0, p1, move_distance, dist(p0, p1));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
        Point shiftPointTowards(Point& p0, Point& p1, const utils::numeric auto move_distance, const utils::floating_point auto p0p1_distance)
    const noexcept
    {
        using coord_type = std::remove_cvref_t<decltype(std::get<"X">(p0))>;
        const auto shift_distance = move_distance / p0p1_distance;
        return {
            std::get<"X">(p0) + static_cast<coord_type>((std::get<"X">(p1) - std::get<"X">(p0)) * shift_distance),
            std::get<"Y">(p0) + static_cast<coord_type>((std::get<"Y">(p1) - std::get<"Y">(p0)) * shift_distance),
        };
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point> utils::floating_point auto dist(Point& point_0, Point& point_1) const noexcept
    {
        return std::hypot(std::get<"X">(point_0) - std::get<"X">(point_1), std::get<"Y">(point_0) - std::get<"Y">(point_1));
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector> utils::floating_point auto magnitude(Vector& v) const noexcept
    {
        return std::hypot(std::get<"X">(v), std::get<"Y">(v));
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    auto dotProduct(Vector& point_0, Vector& point_1) const noexcept
    {
        return std::get<"X">(point_0) * std::get<"X">(point_1) + std::get<"Y">(point_0) * std::get<"Y">(point_1);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    bool isSmooth(Point& A, Point& B, Point& C, Point& D, utils::floating_point auto fluid_motion_angle) const noexcept
    {
        /*
         * Move points A and B, so they are both at equal distance from C and D
         *
         *        B--C
         *       /    \
         *      A_     D_
         *     /        \
         *    /          \
         *   A            \
         *                 \
         *                  D
         *
         * Points B, C are in a "fluid motion" with points A, D if
         * vectors [A_,D_] and [B,C] are oriented within a certain angle
         */
        constexpr auto shift_distance = 300.;
        auto A_ = shiftPointTowards(B, A, shift_distance);
        auto D_ = shiftPointTowards(C, D, shift_distance);

        // precompute distance BC
        const auto BC_magnitude = dist(B, C);

        const auto cos_angle_fluid = cosAngle(A_, D_, B, C, dist(A_, D_), BC_magnitude);
        const auto cos_angle_abc = cosAngle(A_, B, C, shift_distance, BC_magnitude);
        const auto cos_angle_bcd = cosAngle(B, C, D_, BC_magnitude, shift_distance);

        // tThe motion is fluid if either of the marker angles is smaller than the max angle
        return cos_angle_fluid >= fluid_motion_angle || cos_angle_abc >= fluid_motion_angle || cos_angle_bcd >= fluid_motion_angle;
    }
};

inline constexpr smooth_fn smooth{};
} // namespace cura::actions

#endif // UTILS_VIEWS_SMOOTH_H
