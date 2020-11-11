//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "DistributedBeadingStrategy.h"

namespace cura
{

DistributedBeadingStrategy::Beading DistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
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
        const auto widen_by = static_cast<float>(thickness) / (optimal_width_outer + optimal_width_inner * (bead_count - 1));

        // Outer wall:
        const auto distributed_width_outer = static_cast<coord_t>(optimal_width_outer * widen_by);
        ret.bead_widths.emplace_back(distributed_width_outer);
        ret.toolpath_locations.emplace_back(distributed_width_outer / 2);

        // Evenly distributed inner walls:
        const coord_t distributed_width_inner = (thickness - distributed_width_outer) / (bead_count - 1);
        for (coord_t bead_idx = 1; bead_idx < bead_count; bead_idx++)
        {
            ret.bead_widths.emplace_back(distributed_width_inner);
            ret.toolpath_locations.emplace_back((distributed_width_outer - distributed_width_inner / 2) + distributed_width_inner * bead_idx);
        }

        ret.left_over = 0; // There should be nothing left over, as we've distributed the remaining space.
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t DistributedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return std::max(0LL, (bead_count - 1)) * optimal_width_inner + std::min(1LL, bead_count) * optimal_width_outer;
}

coord_t DistributedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    // TODO: doesnt take min and max width into account
    const coord_t optimal_thickness = this->getOptimalThickness(lower_bead_count);
    return optimal_thickness + (lower_bead_count < 1 ? optimal_width_outer : optimal_width_inner) / 2;
}

coord_t DistributedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    coord_t count = std::min(1LL, (thickness + optimal_width_outer / 2) / optimal_width_outer);
    const coord_t thickness_left = thickness - count * optimal_width_outer;
    count += (thickness_left + (optimal_width_inner / 2)) / optimal_width_inner;
    return count;
}

} // namespace cura
