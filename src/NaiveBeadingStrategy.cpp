//Copyright (c) 2019 Ultimaker B.V.


#include "NaiveBeadingStrategy.h"

namespace arachne
{

NaiveBeadingStrategy::Beading NaiveBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        ret.bead_widths.resize(bead_count, optimal_width);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            ret.toolpath_locations.emplace_back(optimal_width * (bead_idx * 2 + 1) / 2);
        }
        ret.left_over = thickness - bead_count * optimal_width;
    }
    else
    {
        ret.left_over = thickness;
    }
    return ret;
}

coord_t NaiveBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t NaiveBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    return (lower_bead_count + 1) * optimal_width - 1; // TODO: this might be incorrect, but it isn't used because the transition angle is set to zero
}

coord_t NaiveBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    return (thickness / 2 + optimal_width / 2) / optimal_width * 2;
}


} // namespace arachne
