// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/ExtrusionSegment.h"

#include <numbers>

#include <spdlog/spdlog.h>

#include "utils/macros.h"

namespace cura
{

Shape ExtrusionSegment::toShape()
{
    return toShape(is_reduced_);
}

Shape ExtrusionSegment::toShape(bool reduced)
{
    Shape ret;
    const Point2LL vec = to_.p_ - from_.p_;
    const coord_t vec_length = vSize(vec);

    if (vec_length <= 0) // Don't even output the endcaps.
    {
        return ret;
    }

    Polygon& poly = ret.newLine();
    const double delta_r = 0.5 * std::abs(from_.w_ - to_.w_);
    const double vec_length_fixed = std::max(delta_r, static_cast<double>(vec_length));
    float alpha = std::acos(delta_r / vec_length_fixed); // Angle between the slope along the edge of the polygon (due to varying line width) and the centerline.
    if (to_.w_ > from_.w_)
    {
        alpha = std::numbers::pi - alpha;
    }
    assert(alpha > -std::numbers::pi - 0.0001);
    assert(alpha < std::numbers::pi + 0.0001);
    if (alpha <= -std::numbers::pi || alpha >= std::numbers::pi)
    {
        RUN_ONCE(spdlog::warn("Line joint slope is out of bounds (should be between -pi and +pi): {}", alpha));
    }

    double dir = std::atan(vec.Y / static_cast<double>(vec.X));
    if (vec.X < 0)
    {
        dir += std::numbers::pi;
    }

    // Draw the endcap on the "from" vertex's end.
    {
        poly.push_back(from_.p_ + Point2LL(from_.w_ / 2 * cos(alpha + dir), from_.w_ / 2 * sin(alpha + dir)));

        double start_a = 2 * std::numbers::pi;
        while (start_a > alpha + dir)
        {
            start_a -= a_step;
        }
        start_a += a_step;

        double end_a = -2 * std::numbers::pi;
        while (end_a < 2 * std::numbers::pi - alpha + dir)
        {
            end_a += a_step;
        }

        // Draw the endcap.
        for (double a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from_.p_ + Point2LL(from_.w_ / 2 * cos(a), from_.w_ / 2 * sin(a)));
        }
        poly.emplace_back(from_.p_ + Point2LL(from_.w_ / 2 * cos(2 * std::numbers::pi - alpha + dir), from_.w_ / 2 * sin(2 * std::numbers::pi - alpha + dir)));
    }

    // Draw the endcap on the "to" vertex's end.
    {
        poly.push_back(
            to_.p_
            + Point2LL(
                to_.w_ / 2 * cos(2 * std::numbers::pi - alpha + dir),
                to_.w_ / 2 * sin(2 * std::numbers::pi - alpha + dir))); // Also draws the main diagonal from the "from" vertex to the "to" vertex!

        double start_a = 2 * std::numbers::pi;
        while (start_a > alpha + dir)
        {
            start_a -= a_step;
        }
        if (reduced)
        {
            // If reduced, we'll go the other way around the circle, drawing the other half.
            // The rounding at the ends works slightly different then.
            start_a += a_step;
        }

        double end_a = -2 * std::numbers::pi;
        while (end_a < 2 * std::numbers::pi - alpha + dir)
            end_a += a_step;
        if (reduced)
        {
            end_a -= a_step;
        }
        else
        {
            end_a -= 2 * std::numbers::pi;
        }

        // Draw the endcap.
        if (reduced)
        {
            for (double a = end_a; a >= start_a; a -= a_step) // Go in the opposite direction.
            {
                poly.emplace_back(to_.p_ + Point2LL(to_.w_ / 2 * cos(a), to_.w_ / 2 * sin(a)));
            }
        }
        else
        {
            for (double a = end_a; a <= start_a; a += a_step)
            {
                poly.emplace_back(to_.p_ + Point2LL(to_.w_ / 2 * cos(a), to_.w_ / 2 * sin(a)));
            }
        }

        poly.emplace_back(to_.p_ + Point2LL(to_.w_ / 2 * cos(alpha + dir), to_.w_ / 2 * sin(alpha + dir)));
        // The other main diagonal from the "to" vertex to the "from" vertex is implicit in the closing of the polygon.
    }

#ifdef DEBUG
    for (const Point2LL& p : poly)
    {
        assert(p.X < 0x3FFFFFFFFFFFFFFFLL);
        assert(p.Y < 0x3FFFFFFFFFFFFFFFLL);
    }
#endif // DEBUG

    return ret;
}


std::vector<ExtrusionSegment> ExtrusionSegment::discretize(coord_t step_size)
{
    Point2LL a = from_.p_;
    Point2LL b = to_.p_;
    Point2LL ab = b - a;
    coord_t ab_length = vSize(ab);
    coord_t step_count = std::max(static_cast<coord_t>(1), (ab_length + step_size / 2) / step_size);
    std::vector<ExtrusionSegment> discretized;
    for (coord_t step = 0; step < step_count; step++)
    {
        ExtrusionJunction mid(a + ab * (step + 1) / step_count, from_.w_ + (to_.w_ - from_.w_) * (step + 1) / step_count, from_.perimeter_index_);
        discretized.emplace_back(from_, mid, is_odd_, true);
        from_ = mid;
    }
    discretized.back().is_reduced_ = is_reduced_;
    return discretized;
}

} // namespace cura
