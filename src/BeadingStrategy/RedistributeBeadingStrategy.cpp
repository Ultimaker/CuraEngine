//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

namespace cura
{
    coord_t countPositive(const std::vector<coord_t>& bead_widths)
    {
        coord_t ret = 0;
        for (const auto& width : bead_widths)
        {
            ret += width > 0 ? 1 : 0;
        }
        return ret;
    }

    BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret = parent->compute(thickness, bead_count);
        bead_count = countPositive(ret.bead_widths); // Don't count any zero-width 'signalling' walls.
        thickness -= ret.left_over;
        if (bead_count < 3)
        {
            return ret;
        }

        const coord_t current_total_outer_walls_width = ret.bead_widths.front() + ret.bead_widths.back();
        const coord_t current_total_inner_walls_width = thickness - current_total_outer_walls_width;
        const float current_outer_factor = current_total_outer_walls_width / static_cast<float>(thickness);
        const float current_inner_factor = current_total_inner_walls_width / static_cast<float>(thickness);

        const coord_t optimal_total_outer_walls_width = optimal_width_outer * 2;
        const coord_t optimal_total_inner_walls_width = optimal_width_inner * (bead_count - 2);
        const float optimal_outer_factor = optimal_total_outer_walls_width / static_cast<float>(optimal_total_outer_walls_width + optimal_total_inner_walls_width);
        const float optimal_inner_factor = optimal_total_inner_walls_width / static_cast<float>(optimal_total_outer_walls_width + optimal_total_inner_walls_width);

        const float outer_factor = optimal_outer_factor / current_outer_factor;
        const float inner_factor = optimal_inner_factor / current_inner_factor;

        ret.bead_widths[0] *= outer_factor;
        for (coord_t i_width = 1; i_width < (bead_count - 1); ++i_width)
        {
            ret.bead_widths[i_width] *= inner_factor;
        }
        ret.bead_widths[bead_count - 1] *= outer_factor;

        coord_t last_coord = 0;
        coord_t last_width = 0;
        for (coord_t i_location = 0; i_location < bead_count / 2; ++i_location)
        {
            ret.toolpath_locations[i_location] = last_coord + (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

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
