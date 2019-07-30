//Copyright (c) 2019 Ultimaker B.V.


#include "SingleBeadBeadingStrategy.h"

namespace arachne
{

SingleBeadBeadingStrategy::Beading SingleBeadBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count == 1)
    {
        ret.bead_widths.emplace_back(thickness);
        ret.toolpath_locations.emplace_back(thickness / 2);
    }
    if (bead_count > 1)
    {
        coord_t width = std::min(optimal_width, thickness / (bead_count - 1));
        ret.bead_widths.resize(bead_count, width);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            ret.toolpath_locations.emplace_back(width * (bead_idx * 2 + 1) / 2);
        }
        ret.left_over = thickness - bead_count * width;
    }
    else
    {
        ret.left_over = thickness;
    }
    return ret;
}

coord_t SingleBeadBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return bead_count * optimal_width - 10;
}

coord_t SingleBeadBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count <= 0) return 0;
    else if (lower_bead_count == 1) return optimal_width;
    else return 9999999;
}

coord_t SingleBeadBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    if (thickness <= optimal_width) return 1;
    else return 2;
}

coord_t SingleBeadBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return 20;
}

} // namespace arachne
