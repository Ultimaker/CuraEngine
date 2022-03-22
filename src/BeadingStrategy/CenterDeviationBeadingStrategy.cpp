// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
#include <algorithm>

#include "CenterDeviationBeadingStrategy.h"

namespace cura
{
CenterDeviationBeadingStrategy::CenterDeviationBeadingStrategy
(
    const coord_t pref_bead_width,
    const AngleRadians transitioning_angle,
    const Ratio wall_split_middle_threshold,
    const Ratio wall_add_middle_threshold
) :
    BeadingStrategy(pref_bead_width, wall_split_middle_threshold, wall_add_middle_threshold, pref_bead_width / 2, transitioning_angle)
{
    name = "CenterDeviationBeadingStrategy";
}

CenterDeviationBeadingStrategy::Beading CenterDeviationBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        // Set the bead widths
        ret.bead_widths = std::vector<coord_t>(static_cast<size_t>(bead_count), optimal_width);
        coord_t leftover_thickness = ret.total_thickness;
        for (size_t bead_index = 0; bead_index <= bead_count / 2; ++bead_index)
        {
            const size_t opposite_bead_index = bead_count - (1 + bead_index);
            switch (opposite_bead_index - bead_index)
            {
            case 0: // Single bead in the middle:
                ret.bead_widths[bead_index] = leftover_thickness;
                break;
            case 1: // Two beads in the middle:
                ret.bead_widths[bead_index] = leftover_thickness / 2;
                ret.bead_widths[opposite_bead_index] = leftover_thickness / 2;
                break;
            default: // Beads on the outside.
                ret.bead_widths[bead_index] = optimal_width;
                ret.bead_widths[opposite_bead_index] = optimal_width;
                break;
            }
            leftover_thickness -= ret.bead_widths[bead_index] + ret.bead_widths[opposite_bead_index]; // Incorrect when there is a last single middle line, but the loop will exit anyway then.
        }

        // Set the center line location of the bead toolpaths.
        ret.toolpath_locations.resize(ret.bead_widths.size());
        ret.toolpath_locations.front() = ret.bead_widths.front() / 2;
        for (size_t bead_idx = 1; bead_idx < ret.bead_widths.size(); ++bead_idx)
        {
            ret.toolpath_locations[bead_idx] =
              ret.toolpath_locations[bead_idx - 1] + (ret.bead_widths[bead_idx] + ret.bead_widths[bead_idx - 1]) / 2;
        }
        ret.left_over = 0;
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t CenterDeviationBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    const coord_t naive_count = thickness / optimal_width; // How many lines we can fit in for sure. (Note difference with Distributed.)
    const coord_t remainder = thickness - naive_count * optimal_width; // Space left after fitting that many lines.
    const coord_t minimum_line_width = optimal_width * (naive_count % 2 == 1 ? wall_split_middle_threshold : wall_add_middle_threshold);
    return naive_count + (remainder >= minimum_line_width); // If there's enough space, fit an extra one.
}

} // namespace cura
