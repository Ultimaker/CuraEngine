// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/ExtrusionSegment.h"

#include <spdlog/spdlog.h>

#include "utils/macros.h"

namespace cura
{

Polygons ExtrusionSegment::toPolygons()
{
    return toPolygons(is_reduced);
}

Polygons ExtrusionSegment::toPolygons(bool reduced)
{
    Polygons ret;
    const Point vec = to.p - from.p;
    const coord_t vec_length = vSize(vec);

    if (vec_length <= 0) // Don't even output the endcaps.
    {
        return ret;
    }

    PolygonRef poly = ret.newPoly();
    const float delta_r = 0.5f * std::abs(from.w - to.w);
    const float vec_length_fixed = std::max(delta_r, static_cast<float>(vec_length));
    float alpha = std::acos(delta_r / vec_length_fixed); // Angle between the slope along the edge of the polygon (due to varying line width) and the centerline.
    if (to.w > from.w)
    {
        alpha = M_PI - alpha;
    }
    assert(alpha > -M_PI - 0.0001);
    assert(alpha < M_PI + 0.0001);
    if (alpha <= -M_PI || alpha >= M_PI)
    {
        RUN_ONCE(spdlog::warn("Line joint slope is out of bounds (should be between -pi and +pi): {}", alpha));
    }

    float dir = std::atan(vec.Y / static_cast<float>(vec.X));
    if (vec.X < 0)
    {
        dir += M_PI;
    }

    // Draw the endcap on the "from" vertex's end.
    {
        poly.emplace_back(from.p + Point(from.w / 2 * cos(alpha + dir), from.w / 2 * sin(alpha + dir)));

        float start_a = 2 * M_PI;
        while (start_a > alpha + dir)
        {
            start_a -= a_step;
        }
        start_a += a_step;

        float end_a = -2 * M_PI;
        while (end_a < 2 * M_PI - alpha + dir)
        {
            end_a += a_step;
        }

        // Draw the endcap.
        for (float a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from.p + Point(from.w / 2 * cos(a), from.w / 2 * sin(a)));
        }
        poly.emplace_back(from.p + Point(from.w / 2 * cos(2 * M_PI - alpha + dir), from.w / 2 * sin(2 * M_PI - alpha + dir)));
    }

    // Draw the endcap on the "to" vertex's end.
    {
        poly.emplace_back(to.p + Point(to.w / 2 * cos(2 * M_PI - alpha + dir), to.w / 2 * sin(2 * M_PI - alpha + dir))); // Also draws the main diagonal from the "from" vertex to the "to" vertex!

        float start_a = 2 * M_PI;
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

        float end_a = -2 * M_PI;
        while (end_a < 2 * M_PI - alpha + dir)
            end_a += a_step;
        if (reduced)
        {
            end_a -= a_step;
        }
        else
        {
            end_a -= 2 * M_PI;
        }

        // Draw the endcap.
        if (reduced)
        {
            for (float a = end_a; a >= start_a; a -= a_step) // Go in the opposite direction.
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }
        else
        {
            for (float a = end_a; a <= start_a; a += a_step)
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }

        poly.emplace_back(to.p + Point(to.w / 2 * cos(alpha + dir), to.w / 2 * sin(alpha + dir)));
        // The other main diagonal from the "to" vertex to the "from" vertex is implicit in the closing of the polygon.
    }

#ifdef DEBUG
    for (Point p : poly)
    {
        assert(p.X < 0x3FFFFFFFFFFFFFFFLL);
        assert(p.Y < 0x3FFFFFFFFFFFFFFFLL);
    }
#endif // DEBUG

    return ret;
}


std::vector<ExtrusionSegment> ExtrusionSegment::discretize(coord_t step_size)
{
    Point a = from.p;
    Point b = to.p;
    Point ab = b - a;
    coord_t ab_length = vSize(ab);
    coord_t step_count = std::max(static_cast<coord_t>(1), (ab_length + step_size / 2) / step_size);
    std::vector<ExtrusionSegment> discretized;
    for (coord_t step = 0; step < step_count; step++)
    {
        ExtrusionJunction mid(a + ab * (step + 1) / step_count, from.w + (to.w - from.w) * (step + 1) / step_count, from.perimeter_index);
        discretized.emplace_back(from, mid, is_odd, true);
        from = mid;
    }
    discretized.back().is_reduced = is_reduced;
    return discretized;
}

} // namespace cura
