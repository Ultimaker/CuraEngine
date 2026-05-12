// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/FibonacciSpiralInfill.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"
#include "utils/Coord_t.h"

namespace cura
{

OpenLinesSet FibonacciSpiralInfill::generate(
    const Shape& outline,
    const coord_t line_distance,
    const coord_t z,
    const double perimeter_start_ratio)
{
    OpenLinesSet result;

    if (outline.empty() || line_distance <= 0)
    {
        return result;
    }

    // ---- Process each closed island independently ----------------------------
    for (const Polygon& island : outline)
    {
        if (island.size() < 3)
        {
            continue;
        }

        // ------------------------------------------------------------------
        // 1. Compute AABB center as the spiral center.
        // ------------------------------------------------------------------
        AABB bb;
        for (const Point2LL& pt : island)
        {
            bb.include(pt);
        }
        const Point2LL center((bb.min_.X + bb.max_.X) / 2, (bb.min_.Y + bb.max_.Y) / 2);

        // ------------------------------------------------------------------
        // 2. Compute the maximum radius (distance from center to farthest vertex).
        //    This is the outer radius at which the spiral starts.
        // ------------------------------------------------------------------
        double r_max = 0.0;
        for (const Point2LL& pt : island)
        {
            const double dx = static_cast<double>(pt.X - center.X);
            const double dy = static_cast<double>(pt.Y - center.Y);
            r_max = std::max(r_max, std::sqrt(dx * dx + dy * dy));
        }

        if (r_max < 1.0)
        {
            continue; // Degenerate island.
        }

        // ------------------------------------------------------------------
        // 3. Determine the spiral entry angle.
        //
        //    We walk `perimeter_start_ratio` of the island perimeter and take
        //    the angle from center to that point as the starting angle of the
        //    spiral.  Then we apply a Z-based offset so consecutive layers
        //    begin at a different angular position, creating inter-layer
        //    crossing (adhesion).
        //
        //    Walking the perimeter: accumulate edge lengths, stop at
        //    target_length = perimeter_start_ratio × total_perimeter.
        // ------------------------------------------------------------------
        double total_perimeter = 0.0;
        {
            for (size_t i = 0; i < island.size(); ++i)
            {
                const Point2LL& a = island[i];
                const Point2LL& b = island[(i + 1) % island.size()];
                const double dx = static_cast<double>(b.X - a.X);
                const double dy = static_cast<double>(b.Y - a.Y);
                total_perimeter += std::sqrt(dx * dx + dy * dy);
            }
        }

        const double target_dist = std::max(0.0, std::min(1.0, perimeter_start_ratio)) * total_perimeter;
        double walked = 0.0;
        Point2LL entry_point = island[0];
        bool found_entry = false;
        for (size_t i = 0; i < island.size(); ++i)
        {
            const Point2LL& a = island[i];
            const Point2LL& b = island[(i + 1) % island.size()];
            const double dx = static_cast<double>(b.X - a.X);
            const double dy = static_cast<double>(b.Y - a.Y);
            const double seg_len = std::sqrt(dx * dx + dy * dy);
            if (walked + seg_len >= target_dist)
            {
                const double t = (target_dist - walked) / std::max(seg_len, 1.0);
                entry_point = Point2LL(
                    static_cast<coord_t>(a.X + t * dx),
                    static_cast<coord_t>(a.Y + t * dy));
                found_entry = true;
                break;
            }
            walked += seg_len;
        }
        if (! found_entry)
        {
            entry_point = island[0]; // fallback: use first vertex
        }

        // Angle from center to entry point.
        double start_angle = std::atan2(
            static_cast<double>(entry_point.Y - center.Y),
            static_cast<double>(entry_point.X - center.X));

        // Z-based angular shift: one full revolution per `line_distance` of Z travel.
        // This simulates a 3-D helix cross-section — each layer starts at a different
        // angular position, ensuring the line from layer N crosses the line from layer N±1.
        const double z_angle_shift = (static_cast<double>(z) / static_cast<double>(line_distance)) * 2.0 * std::numbers::pi;
        start_angle += z_angle_shift;

        // ------------------------------------------------------------------
        // 4. Generate the inward Archimedean spiral.
        //
        //    r(θ) = r_max − b × Δθ      where  b = line_distance / (2π)
        //
        //    We step in small angular increments (≈ line_width equivalent,
        //    set to line_distance/10 for PoC) and stop when r ≤ 0.
        // ------------------------------------------------------------------
        const double b = static_cast<double>(line_distance) / (2.0 * std::numbers::pi);
        // Angular step: produce ~360/steps_per_turn points per revolution.
        // A reasonable value: aim for point spacing ≈ line_distance/8 along the arc.
        // At radius r, arc length per radian = r, so delta_theta ≈ (line_distance/8)/r_max
        // but we clamp to a sensible range [π/180, π/18] (1°–10°).
        const double step_angle = std::clamp(
            static_cast<double>(line_distance) / (8.0 * r_max),
            std::numbers::pi / 180.0,    // min 1°
            std::numbers::pi / 18.0);    // max 10°

        OpenPolyline spiral;
        double theta = 0.0; // angle advance from start_angle
        while (true)
        {
            const double r = r_max - b * theta;
            if (r <= 0.0)
            {
                break;
            }
            const double angle = start_angle - theta; // spiral inward (clockwise)
            const coord_t x = center.X + static_cast<coord_t>(r * std::cos(angle));
            const coord_t y = center.Y + static_cast<coord_t>(r * std::sin(angle));
            spiral.push_back(Point2LL(x, y));
            theta += step_angle;
        }
        // Add center as endpoint so the spiral terminates cleanly.
        spiral.push_back(center);

        if (spiral.size() < 2)
        {
            continue;
        }

        // ------------------------------------------------------------------
        // 5. Clip the spiral to the island outline.
        //    Build a shape from the island polygon, then intersect.
        // ------------------------------------------------------------------
        Shape island_shape;
        island_shape.push_back(island);

        OpenLinesSet raw;
        raw.push_back(spiral);
        const OpenLinesSet clipped = island_shape.intersection(raw);

        result.push_back(clipped);
    }

    return result;
}

} // namespace cura
