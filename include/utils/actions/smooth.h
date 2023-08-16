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
#include <limits>
#include <numbers>
#include <set>

namespace cura
{
class SmoothTest_TestSmooth_Test;
} // namespace cura

namespace cura::actions
{

struct smooth_fn
{
    friend class cura::SmoothTest_TestSmooth_Test;

    auto operator()(const Settings& settings) const
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
        const auto size = ranges::distance(rng) - 1;
        if (size < 4)
        {
            return static_cast<Rng&&>(rng);
        }

        const auto fluid_motion_shift_distance3 = 3 * fluid_motion_shift_distance;
        const auto cos_fluid_motion_angle = std::cos(fluid_motion_angle);

        auto tmp = rng; // We don't want to shift the points of the in-going range, therefore we create a temporary copy
        auto windows = ranges::views::concat(ranges::views::single(ranges::back(tmp)), ranges::views::concat(tmp, tmp | ranges::views::take(4))) | ranges::views::addressof;

        // Smooth the path, by moving over three segments at a time. If the middle segment is shorter than the max resolution, then we try shifting those points outwards.
        // The previous and next segment should have a remaining length of at least the smooth distance, otherwise the point is not shifted, but deleted.
        for (auto windows_it = ranges::begin(windows); ranges::distance(windows_it, ranges::end(windows)) > 2; ++windows_it)
        {
            const auto a = *windows_it;
            const auto b = *std::next(windows_it, 1);
            const auto c = *std::next(windows_it, 2);
            const auto d = *std::next(windows_it, 3);

            const auto magnitude_ab = dist(*a, *b);
            const auto magnitude_bc = dist(*b, *c);
            const auto magnitude_cd = dist(*c, *d);

            if (magnitude_bc > fluid_motion_small_distance)
            {
                continue;
            }

            // only if segments ab and cd are long enough, we can shift b and c
            // 3 * fluid_motion_shift_distance is the minimum length of the segments ab and cd
            // as this allows us to shift both ends with the shift distance and still have some room to spare
            if (magnitude_ab < fluid_motion_shift_distance3 || magnitude_cd < fluid_motion_shift_distance3)
            {
                continue;
            }

            if (! isSmooth(*a, *b, *c, *d, cos_fluid_motion_angle, magnitude_ab, magnitude_bc, magnitude_cd))
            {
                *b = shiftPointTowards(*b, *a, fluid_motion_shift_distance, magnitude_ab);
                *c = shiftPointTowards(*c, *d, fluid_motion_shift_distance, magnitude_cd);
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
    inline constexpr auto cosAngle(Point& a, Point& b, Point& c) const noexcept
    {
        return cosAngle(a, b, c, dist(a, b), dist(b, c));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    inline constexpr auto cosAngle(Point& a, Point& b, Point& c, const utils::floating_point auto ab_magnitude, const utils::floating_point auto bc_magnitude) const noexcept
    {
        return cosAngle(a, b, b, c, ab_magnitude, bc_magnitude);
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
    inline constexpr auto cosAngle(Point& a, Point& b, Point& c, Point& d) const noexcept
    {
        return cosAngle(a, b, c, d, dist(a, b), dist(c, d));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    inline constexpr auto
        cosAngle(Point& a, Point& b, Point& c, Point& d, const utils::floating_point auto ab_magnitude, const utils::floating_point auto bc_magnitude) const noexcept
    {
        Point vector_ab = { std::get<"X">(b) - std::get<"X">(a), std::get<"Y">(b) - std::get<"Y">(a) };
        Point vector_cd = { std::get<"X">(d) - std::get<"X">(c), std::get<"Y">(d) - std::get<"Y">(c) };

        return cosAngle(vector_ab, vector_cd, ab_magnitude, bc_magnitude);
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
    inline constexpr auto cosAngle(Vector& a, Vector& b) const noexcept
    {
        return cosAngle<Point>(a, b, magnitude(a), magnitude(b));
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    inline constexpr auto cosAngle(Vector& a, Vector& b, const utils::floating_point auto a_magnitude, const utils::floating_point auto b_magnitude) const noexcept
    {
        if (a_magnitude <= std::numeric_limits<decltype(a_magnitude)>::epsilon() || b_magnitude <= std::numeric_limits<decltype(b_magnitude)>::epsilon())
        {
            return static_cast<decltype(a_magnitude * b_magnitude)>(0.0);
        }
        return static_cast<decltype(a_magnitude * b_magnitude)>(dotProduct(a, b)) / (a_magnitude * b_magnitude);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    inline constexpr Point shiftPointTowards(Point& p0, Point& p1, const utils::numeric auto move_distance) const noexcept
    {
        return shiftPointTowards(p0, p1, move_distance, dist(p0, p1));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    inline constexpr Point shiftPointTowards(Point& p0, Point& p1, const utils::numeric auto move_distance, const utils::floating_point auto p0p1_distance) const noexcept
    {
        using coord_type = std::remove_cvref_t<decltype(std::get<"X">(p0))>;
        const auto shift_distance = move_distance / p0p1_distance;
        return {
            std::get<"X">(p0) + static_cast<coord_type>((std::get<"X">(p1) - std::get<"X">(p0)) * shift_distance),
            std::get<"Y">(p0) + static_cast<coord_type>((std::get<"Y">(p1) - std::get<"Y">(p0)) * shift_distance),
        };
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    inline constexpr utils::floating_point auto dist(Point& point_0, Point& point_1) const noexcept
    {
        Point vector = { std::get<"X">(point_1) - std::get<"X">(point_0), std::get<"Y">(point_1) - std::get<"Y">(point_0) };
        return magnitude(vector);
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    inline constexpr utils::floating_point auto magnitude(Vector& v) const noexcept
    {
        return std::hypot(std::get<"X">(v), std::get<"Y">(v));
    }

    template<class Vector>
    requires utils::point2d<Vector> || utils::junction<Vector>
    inline constexpr auto dotProduct(Vector& point_0, Vector& point_1) const noexcept
    {
        return std::get<"X">(point_0) * std::get<"X">(point_1) + std::get<"Y">(point_0) * std::get<"Y">(point_1);
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr bool isSmooth(Point& a, Point& b, Point& c, Point& d, const utils::floating_point auto fluid_motion_angle) const noexcept
    {
        return isSmooth(a, b, c, d, fluid_motion_angle, dist(a, b), dist(b, c), dist(c, d));
    }

    template<class Point>
    requires utils::point2d<Point> || utils::junction<Point>
    constexpr bool isSmooth(
        Point& a,
        Point& b,
        Point& c,
        Point& d,
        const utils::floating_point auto fluid_motion_angle,
        const utils::floating_point auto dist_ab,
        const utils::floating_point auto dist_bc,
        const utils::floating_point auto dist_cd) const noexcept
    {
        /*
         * Move points A and D, so they are both at equal distance from B and C
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
        auto a_ = shiftPointTowards(b, a, shift_distance, dist_ab);
        auto d_ = shiftPointTowards(c, d, shift_distance, dist_cd);

        const auto cos_angle_fluid = cosAngle(a_, d_, b, c, dist(a_, d_), dist_bc);
        const auto cos_angle_abc = cosAngle(a_, b, c, shift_distance, dist_bc);
        const auto cos_angle_bcd = cosAngle(b, c, d_, dist_bc, shift_distance);

        // The motion is fluid if either of the marker angles is smaller than the max angle
        return cos_angle_fluid >= fluid_motion_angle || cos_angle_abc >= fluid_motion_angle || cos_angle_bcd >= fluid_motion_angle;
    }
};

inline constexpr smooth_fn smooth{};
} // namespace cura::actions

#endif // UTILS_VIEWS_SMOOTH_H
