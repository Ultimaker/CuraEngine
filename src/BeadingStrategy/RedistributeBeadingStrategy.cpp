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
        const coord_t current_total_outer_walls_width = ret.bead_widths.front() + ret.bead_widths.back();
        const coord_t current_total_inner_walls_width = thickness - current_total_outer_walls_width;
        const coord_t optimal_total_outer_walls_width = optimal_width_outer * 2;
        const coord_t optimal_total_inner_walls_width = optimal_width_inner * (bead_count - 2);

        const coord_t outer_factor_numerator = optimal_total_outer_walls_width * thickness;
        const coord_t outer_factor_denominator = current_total_outer_walls_width * (optimal_total_outer_walls_width + optimal_total_inner_walls_width);
        const coord_t inner_factor_numerator = optimal_total_inner_walls_width * thickness;
        const coord_t inner_factor_denominator = current_total_inner_walls_width * (optimal_total_outer_walls_width + optimal_total_inner_walls_width);

        // Multiply the bead-widths with the right factors:
        ret.bead_widths[0] = (ret.bead_widths[0] * outer_factor_numerator) / outer_factor_denominator;
        for (coord_t i_width = 1; i_width < (bead_count - 1); ++i_width)
        {
            ret.bead_widths[i_width] = (ret.bead_widths[i_width] * inner_factor_numerator) / inner_factor_denominator;
        }
        ret.bead_widths[bead_count - 1] = (ret.bead_widths[bead_count - 1] * outer_factor_numerator) / outer_factor_denominator;

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
