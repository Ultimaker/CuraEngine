// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
#include <algorithm>

#include "CenterDeviationBeadingStrategy.h"

namespace cura
{
CenterDeviationBeadingStrategy::CenterDeviationBeadingStrategy(const coord_t pref_bead_width,
                                                               const AngleRadians transitioning_angle,
                                                               const Ratio wall_split_middle_threshold,
                                                               const Ratio wall_add_middle_threshold)
  : BeadingStrategy(pref_bead_width, pref_bead_width / 2, transitioning_angle),
    minimum_line_width_split(pref_bead_width * wall_split_middle_threshold),
    minimum_line_width_add(pref_bead_width * wall_add_middle_threshold)
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
        const coord_t optimal_thickness = getOptimalThickness(bead_count);
        const coord_t diff_thickness = (thickness - optimal_thickness) / 2;
        const coord_t inner_bead_widths = optimal_width + diff_thickness;
        const size_t center_bead_idx = ret.bead_widths.size() / 2;
        if (bead_count % 2 == 0) // Even lines
        {
            if (inner_bead_widths < minimum_line_width_add)
            {
                return compute(thickness, bead_count - 1);
            }
            ret.bead_widths[center_bead_idx - 1] = inner_bead_widths;
            ret.bead_widths[center_bead_idx] = inner_bead_widths;
        }
        else // Uneven lines
        {
            if (inner_bead_widths < minimum_line_width_split)
            {
                return compute(thickness, bead_count - 1);
            }
            ret.bead_widths[center_bead_idx] = inner_bead_widths;
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

coord_t CenterDeviationBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t CenterDeviationBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    return lower_bead_count * optimal_width + (lower_bead_count % 2 == 1 ? minimum_line_width_split : minimum_line_width_add);
}

coord_t CenterDeviationBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    const coord_t naive_count = thickness / optimal_width; // How many lines we can fit in for sure.
    const coord_t remainder = thickness - naive_count * optimal_width; // Space left after fitting that many lines.
    const coord_t minimum_line_width = naive_count % 2 == 1 ? minimum_line_width_split : minimum_line_width_add;
    return naive_count + (remainder > minimum_line_width); // If there's enough space, fit an extra one.
}

} // namespace cura
