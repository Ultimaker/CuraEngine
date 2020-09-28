//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InwardDistributedBeadingStrategy.h"

namespace cura
{

InwardDistributedBeadingStrategy::Beading InwardDistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count == 1)
    {
        ret.bead_widths.emplace_back(thickness);
        ret.toolpath_locations.emplace_back(thickness / 2);
        ret.left_over = 0;
    }
    else if (bead_count > 1)
    {
        const float widen_by = static_cast<float>(thickness) / (optimal_width_outer + optimal_width_inner * (bead_count));

        // Outer wall:
        const coord_t distributed_width_outer = static_cast<coord_t>(optimal_width_outer * widen_by);
        ret.bead_widths.emplace_back(distributed_width_outer);
        ret.toolpath_locations.emplace_back(distributed_width_outer / 2);

        // Inwardly distributed inner walls:
        const coord_t distributed_width_inner = (thickness - distributed_width_outer) / (bead_count - 1);
        const coord_t to_be_divided = thickness - (distributed_width_outer + distributed_width_inner * (bead_count - 1));

        float total_weight = 0;
        const float middle = static_cast<float>(bead_count - 2) / 2;
        
        auto getWeight = [middle, this](coord_t bead_idx)
        {
            float dev_from_middle = (bead_idx - 1) - middle;
            return std::max(0.0f, 1.0f - one_over_distribution_radius_squared * dev_from_middle * dev_from_middle);
        };
        
        for (coord_t bead_idx = 1; bead_idx < bead_count; bead_idx++)
        {
            total_weight += getWeight(bead_idx);
        }

        coord_t half_last_width = ret.toolpath_locations.back();
        for (coord_t bead_idx = 1; bead_idx < bead_count; bead_idx++)
        {
            coord_t width = distributed_width_inner + (to_be_divided * getWeight(bead_idx)) / total_weight;
            ret.bead_widths.emplace_back(width);
            ret.toolpath_locations.emplace_back(ret.toolpath_locations.back() + half_last_width + width / 2);
            half_last_width = width / 2;
        }

        ret.left_over = 0; // There should be nothing left over, as we've distributed the remaining space.
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

} // namespace cura
