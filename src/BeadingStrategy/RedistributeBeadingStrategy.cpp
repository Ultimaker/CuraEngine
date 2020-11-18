//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

#include <algorithm>

namespace cura
{
    BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret = parent->compute(thickness, bead_count);

        // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signalling' walls.
        bead_count = std::count_if(ret.bead_widths.begin(), ret.bead_widths.end(), [](const coord_t width) { return width > 0; });
        thickness -= ret.left_over;

        // Early out when the only walls are outer, the parent can have been trusted to handle it.
        if (bead_count < 3)
        {
            return ret;
        }

        // Calculate the factors with which to multiply the outer and inner walls:
        // (There may be a way to do this with all-integer math, but the speedup won't probably be that much and this will be easier to maintain).
        const float current_total_outer_walls_width = ret.bead_widths.front() + ret.bead_widths.back();
        const float current_total_inner_walls_width = thickness - current_total_outer_walls_width;
        const float current_outer_factor = current_total_outer_walls_width / thickness;
        const float current_inner_factor = current_total_inner_walls_width / thickness;

        const float optimal_total_outer_walls_width = optimal_width_outer * 2;
        const float optimal_total_inner_walls_width = optimal_width_inner * (bead_count - 2);
        const float optimal_outer_factor = optimal_total_outer_walls_width / (optimal_total_outer_walls_width + optimal_total_inner_walls_width);
        const float optimal_inner_factor = optimal_total_inner_walls_width / (optimal_total_outer_walls_width + optimal_total_inner_walls_width);

        const float outer_factor = optimal_outer_factor / current_outer_factor;
        const float inner_factor = optimal_inner_factor / current_inner_factor;

        // Multiply the bead-widths with the right factors:
        ret.bead_widths[0] *= outer_factor;
        for (coord_t i_width = 1; i_width < (bead_count - 1); ++i_width)
        {
            ret.bead_widths[i_width] *= inner_factor;
        }
        ret.bead_widths[bead_count - 1] *= outer_factor;

        // Update the first half of the toolpath-locations with the updated bead-widths (starting from 0, up to half):
        coord_t last_coord = 0;
        coord_t last_width = 0;
        for (coord_t i_location = 0; i_location < bead_count / 2; ++i_location)
        {
            ret.toolpath_locations[i_location] = last_coord + (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

        // NOTE: Don't have to alter the middle toolpath if there's any, it'll already be at half thickness.

        // Update the last half of the toolpath-locations with the updated bead-widths (starting from thickness, down to half):
        last_coord = thickness;
        last_width = 0;
        for (coord_t i_location = bead_count - 1; i_location >= bead_count - (bead_count / 2); --i_location)
        {
            ret.toolpath_locations[i_location] = last_coord - (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

        return ret;
    }

} // namespace cura
