//Copyright (c) 2019 Ultimaker B.V.


#include "CenterDeviationBeadingStrategy.h"

namespace arachne
{

CenterDeviationBeadingStrategy::Beading CenterDeviationBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        for (coord_t bead_idx = 0; bead_idx < bead_count / 2; bead_idx++)
        {
            ret.bead_widths.emplace_back(optimal_width);
            ret.toolpath_locations.emplace_back(optimal_width * (bead_idx * 2 + 1) / 2);
        }
        if (bead_count % 2 == 1)
        {
            ret.bead_widths.emplace_back(thickness - (bead_count - 1) * optimal_width);
            ret.toolpath_locations.emplace_back(thickness / 2);
            ret.left_over = 0;
        }
        else
        {
            ret.left_over = thickness - bead_count * optimal_width;
        }
        for (coord_t bead_idx = (bead_count + 1) / 2; bead_idx < bead_count; bead_idx++)
        {
            ret.bead_widths.emplace_back(optimal_width);
            ret.toolpath_locations.emplace_back(thickness - (bead_count - bead_idx) * optimal_width + optimal_width / 2);
        }
    }
    else
    {
        ret.left_over = thickness;
    }
    return ret;
}

coord_t CenterDeviationBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t CenterDeviationBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count % 2 == 0)
    { // when we add the extra bead in the middle
        return lower_bead_count * optimal_width + underfill_bound;
    }
    else
    { // when we move away from the strategy which replaces two beads by a single one in the middle
        return (lower_bead_count + 1) * optimal_width - overfill_bound;
    }
}

coord_t CenterDeviationBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    coord_t naive_count = (thickness / 2 + optimal_width / 2) / optimal_width * 2;
    coord_t optimal_thickness = naive_count * optimal_width;
    coord_t overfill = optimal_thickness - thickness;
    if (overfill > overfill_bound)
    {
        return naive_count - 1;
    }
    else if (-overfill > underfill_bound)
    {
        return naive_count + 1;
    }
    else
    {
        return naive_count;
    }
}


} // namespace arachne
