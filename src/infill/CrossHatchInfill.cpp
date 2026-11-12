// Copyright (c) 2026 CuraEngine contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/CrossHatchInfill.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"
#include "utils/OpenPolylineStitcher.h"

namespace cura
{

void CrossHatchInfill::generateInfill(
    OpenLinesSet& result_polylines,
    Shape& result_polygons,
    const bool zig_zaggify,
    const coord_t line_distance,
    const Shape& in_outline,
    const coord_t z,
    const coord_t line_width,
    const AngleDegrees& rotation) const
{
    OpenLinesSet generated_lines;
    Shape generated_polygons;
    AbstractLinesInfill::generateInfill(generated_lines, generated_polygons, zig_zaggify, line_distance, in_outline, z, line_width, rotation);
    for (const Polygon& polygon : generated_polygons)
    {
        generated_lines.push_back(polygon.toPseudoOpenPolyline());
    }
    // Boundary connections and rotation both round to integer coordinates.
    // Clip again in the original frame so a rounded connector cannot cut into
    // a hole or notch. Keep existing islands in the output untouched.
    const OpenLinesSet clipped_lines = in_outline.intersection(generated_lines, false, 0, false);
    OpenPolylineStitcher::stitch(clipped_lines, result_polylines, result_polygons, 1);
}

OpenLinesSet CrossHatchInfill::generateParallelLines(const coord_t line_distance, const AABB& bounding_box, const coord_t z, const coord_t line_width) const
{
    if (line_distance <= 0 || line_width <= 0)
    {
        return {};
    }

    const double density = std::clamp(static_cast<double>(line_width) / line_distance, 0.0, 1.0);
    const double repeat_ratio = density < 0.3 ? std::clamp(1.0 - std::exp(-5.0 * density), 0.2, 1.0) : 1.0;
    // Spread the change over enough height that even relatively thick layers
    // can follow the shared diagonals without stepping outside the line width.
    constexpr double transition_ratio = 0.8;
    // A transition's path length, relative to a straight line, is
    // p/2 + sqrt(1 - p + p*p/2). Its mean over p in [0, 1] is given below.
    // Compensate for that extra length over the complete repeat/transition
    // cycle, so the continuous transition preserves the requested density.
    const double mean_transition_length = 0.75 + std::asinh(1.0) / (2.0 * std::sqrt(2.0));
    const double spacing_factor = (repeat_ratio + transition_ratio * mean_transition_length) / (repeat_ratio + transition_ratio);
    const coord_t spacing = std::max(line_width, static_cast<coord_t>(std::llrint(line_distance * spacing_factor)));
    const double repeat_height = spacing * repeat_ratio;
    const double transition_height = spacing * transition_ratio;
    const double half_period = repeat_height + transition_height;
    const double shifted_z = z + repeat_height / 2.0 + transition_height;
    double phase = std::fmod(shifted_z, 2.0 * half_period);
    if (phase < 0)
    {
        phase += 2.0 * half_period;
    }

    bool vertical = phase < half_period;
    const double transition_z = std::fmod(phase, half_period) - repeat_height;
    double progress = 0;
    if (transition_z >= 0)
    {
        const double fraction = transition_z / transition_height;
        if (fraction < 0.5)
        {
            progress = fraction * 2.0;
        }
        else
        {
            progress = (1.0 - fraction) * 2.0;
            vertical = ! vertical;
        }
    }
    // At progress 1 the two orientations share the same 45-degree diagonals.
    // Overshooting this value displaces those diagonals by a fraction of the
    // cell size, leaving inter-layer gaps that grow as density decreases.

    // Anchor to a shared grid, rather than each island's bounding box, so that
    // splitting an infill region cannot move the pattern between layers.
    const coord_t grid = spacing * 4;
    const Point2LL origin(
        static_cast<coord_t>(std::floor(static_cast<double>(bounding_box.min_.X) / grid)) * grid,
        static_cast<coord_t>(std::floor(static_cast<double>(bounding_box.min_.Y) / grid)) * grid);
    const coord_t width = vertical ? bounding_box.max_.Y - origin.Y : bounding_box.max_.X - origin.X;
    const coord_t height = vertical ? bounding_box.max_.X - origin.X : bounding_box.max_.Y - origin.Y;
    const auto to_world = [&](const double x, const double y)
    {
        const coord_t ix = std::llrint(x);
        const coord_t iy = std::llrint(y);
        return origin + (vertical ? Point2LL(iy, ix) : Point2LL(ix, iy));
    };

    OpenLinesSet lines;
    for (coord_t row = -1; row <= height / spacing + 1; ++row)
    {
        OpenPolyline line;
        const coord_t y = row * spacing;
        if (progress == 0)
        {
            line.push_back(to_world(-spacing, y));
            line.push_back(to_world(width + spacing, y));
        }
        else
        {
            const coord_t period = 2 * spacing;
            const double offset = progress * period / 8.0;
            const std::array<std::pair<double, double>, 4> cycle{
                { { period * 0.25 - offset, offset }, { period * 0.25 + offset, offset }, { period * 0.75 - offset, -offset }, { period * 0.75 + offset, -offset } }
            };
            const coord_t shift = row % 2 == 0 ? 0 : -spacing;
            for (coord_t column = -1; column <= width / period + 1; ++column)
            {
                for (const auto& [x_offset, y_offset] : cycle)
                {
                    const Point2LL point = to_world(column * period + shift + x_offset, y + y_offset);
                    if (line.empty() || line.back() != point)
                    {
                        line.push_back(point);
                    }
                }
            }
        }
        lines.push_back(std::move(line));
    }
    return lines;
}

} // namespace cura
