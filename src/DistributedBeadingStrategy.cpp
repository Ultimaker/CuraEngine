//Copyright (c) 2019 Ultimaker B.V.


#include "DistributedBeadingStrategy.h"

namespace arachne
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

coord_t DistributedBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t DistributedBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    return lower_bead_count * optimal_width + optimal_width / 2; // TODO: doesnt take min and max width into account
}

coord_t DistributedBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    return (thickness + optimal_width / 2) / optimal_width;
}

coord_t DistributedBeadingStrategy::getTransitioningLength(coord_t dR, coord_t dd) const
{
    return optimal_width;
}


} // namespace arachne
