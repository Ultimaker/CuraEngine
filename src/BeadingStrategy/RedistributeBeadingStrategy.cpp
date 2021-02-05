//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

#include <algorithm>

namespace cura
{
    // TODO: this needs to be a (class) parameter, want to make it a setting too:
    constexpr float outer_wall_lock_factor = /*0.0; //*/ 1.0;

    BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret = parent->compute(thickness, bead_count);

        // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signalling' walls.
        bead_count = std::count_if(ret.bead_widths.begin(), ret.bead_widths.end(), [](const coord_t width) { return width > 0; });

        // Early out when the only walls are outer, the parent can have been trusted to handle it.
        if (bead_count < 3)
        {
            // TODO: Outer wall lock compensation within this case (this is where it's most noticable after all ... furthermore, the line-thckness will jump if the lock is already implemented for >= 3).
            return ret;
        }

        // Calculate the factors with which to multiply the outer and inner walls:
        const coord_t current_total_outer_walls_width = ret.bead_widths.front() + ret.bead_widths.back();
        const coord_t current_total_inner_walls_width = thickness - current_total_outer_walls_width;
        const coord_t optimal_total_outer_walls_width = optimal_width_outer * 2;
        const coord_t optimal_total_inner_walls_width = optimal_width_inner * (bead_count - 2);

        // ... do some compensation for the 'outer wall lock factor' in between the calculations:
        // NOTE: The weights are inverted here, since these widhts end up in the denominator.
        const coord_t optimal_total_thickness = static_cast<coord_t>(outer_wall_lock_factor * thickness + (1.0 - outer_wall_lock_factor) * (optimal_total_outer_walls_width + optimal_total_inner_walls_width));

        const coord_t outer_factor_numerator = optimal_total_outer_walls_width * thickness;
        const coord_t outer_factor_denominator = current_total_outer_walls_width * optimal_total_thickness;
        const coord_t inner_factor_numerator = optimal_total_inner_walls_width * thickness;
        const coord_t inner_factor_denominator = current_total_inner_walls_width * optimal_total_thickness;

        // TODO: early out if outer walls are already more than the current thickness (could this happen _before_ too? -> well it will be solved then anyway).

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

        // NOTE: The middle toolpath --if there's any-- doesn't have to be altered; it'll already be at the half thickness mark, give or take 1 micron.

        // Update the last half of the toolpath-locations with the updated bead-widths (starting from thickness, down to half):
        last_coord = thickness;
        last_width = 0;
        for (coord_t i_location = bead_count - 1; i_location >= bead_count - (bead_count / 2); --i_location)
        {
            ret.toolpath_locations[i_location] = last_coord - (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

        ret.left_over = std::max(static_cast<coord_t>(0), thickness - (ret.toolpath_locations.back() + ret.bead_widths.back() / 2));
        return ret;
    }

} // namespace cura
