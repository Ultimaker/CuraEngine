// Copyright (c) 2026 CuraEngine contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/CrossHatchInfill.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "geometry/OpenLinesSet.h"
#include "utils/AABB.h"

namespace cura
{

OpenLinesSet CrossHatchInfill::generateParallelLines(const coord_t line_distance, const AABB& bounding_box, const coord_t z, const coord_t line_width) const
{
    if (line_distance <= 0 || line_width <= 0)
    {
        return {};
    }

    const double density = std::clamp(static_cast<double>(line_width) / line_distance, 0.0, 1.0);
    // Transition layers contain more material than straight layers. Match the
    // spacing compensation used by the original Cross Hatch implementation.
    const coord_t spacing = std::max(line_width, static_cast<coord_t>(std::llrint(line_distance * (density < 0.999 ? 1.08 : 1.0))));
    const double repeat_ratio = density < 0.3 ? std::clamp(1.0 - std::exp(-5.0 * density), 0.2, 1.0) : 1.0;
    const double repeat_height = spacing * repeat_ratio;
    const double transition_height = spacing * 0.4;
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
            progress = (fraction + 0.1) * 2.0;
        }
        else
        {
            progress = (1.1 - fraction) * 2.0;
            vertical = ! vertical;
        }
    }

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
        if (transition_z < 0)
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
                    line.push_back(to_world(column * period + shift + x_offset, y + y_offset));
                }
            }
        }
        lines.push_back(std::move(line));
    }
    return lines;
}

} // namespace cura
