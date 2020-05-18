//Copyright (c) 2019 Ultimaker B.V.


#include "ConstantBeadingStrategy.h"

namespace arachne
{

ConstantBeadingStrategy::Beading ConstantBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        coord_t width = thickness / bead_count;
        ret.bead_widths.resize(bead_count, width);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            ret.toolpath_locations.emplace_back(thickness * (bead_idx * 2 + 1) / 2 / bead_count);
        }
        ret.left_over = 0;
    }
    else
    {
        ret.left_over = thickness;
    }
    return ret;
}

coord_t ConstantBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t ConstantBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < bead_count)
    {
        return 0;
    }
    else
    {
        return 999999;
    }
}

coord_t ConstantBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    return bead_count;
}


} // namespace arachne
