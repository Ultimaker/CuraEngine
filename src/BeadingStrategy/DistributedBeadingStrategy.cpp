//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "DistributedBeadingStrategy.h"

namespace cura
{

DistributedBeadingStrategy::Beading DistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        ret.bead_widths.resize(bead_count, thickness / bead_count);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            ret.toolpath_locations.emplace_back(thickness * (bead_idx * 2 + 1) / bead_count / 2);
        }
        ret.left_over = 0;
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t DistributedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return (bead_count - 2) * optimal_width_inner + std::min(2LL, bead_count) * optimal_width_outer;
}

coord_t DistributedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    // TODO: doesnt take min and max width into account
    const coord_t optimal_thickness = this->getOptimalThickness(lower_bead_count);
    return optimal_thickness + (optimal_thickness < 2 ? optimal_width_outer : optimal_width_inner) / 2;
}

coord_t DistributedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    coord_t thickness_left = thickness;
    coord_t count = 0;
    count += std::min(2LL, (thickness + optimal_width_outer / 2) / optimal_width_outer);
    thickness_left -= count * optimal_width_outer;
    if ((optimal_width_inner / 2) >= thickness_left)
    {
        count += (thickness_left + optimal_width_inner / 2) / optimal_width_inner;
    }
    return count;
}

} // namespace cura
