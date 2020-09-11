//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "CenterDeviationBeadingStrategy.h"

namespace cura
{

CenterDeviationBeadingStrategy::Beading CenterDeviationBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count == 1)
    {
        ret.bead_widths.emplace_back(thickness);
        ret.toolpath_locations.emplace_back(thickness / 2);
    }
    else if (bead_count > 1)
    {
        // first half minus middle:
        ret.bead_widths.emplace_back(optimal_width_outer);
        ret.toolpath_locations.emplace_back(optimal_width_outer / 2);
        for (coord_t bead_idx = 1; bead_idx < bead_count / 2; bead_idx++)
        {
            ret.bead_widths.emplace_back(optimal_width_inner);
            ret.toolpath_locations.emplace_back(optimal_width_outer + optimal_width_inner * ((bead_idx - 1) * 2 + 1) / 2);
        }

        // middle (if any):
        const coord_t optimal_width_middle = bead_count == 1 ? optimal_width_outer : optimal_width_inner;
        if (bead_count % 2 == 1)
        {
            ret.bead_widths.emplace_back(thickness - (optimal_width_outer + (bead_count - 2) * optimal_width_inner));
            ret.toolpath_locations.emplace_back(thickness / 2);
            ret.left_over = 0;
        }
        else
        {
            ret.left_over = thickness - bead_count * optimal_width_middle;
        }

        // last half minus middle:
        for (coord_t bead_idx = (bead_count + 1) / 2; bead_idx < bead_count; bead_idx++)
        {
            ret.bead_widths.emplace_back(optimal_width_inner);
            ret.toolpath_locations.emplace_back(thickness - (optimal_width_outer + ((bead_count - bead_idx) - 1) * optimal_width_inner) + optimal_width_inner / 2);
        }
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t CenterDeviationBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return std::max(0LL, (bead_count - 2)) * optimal_width_inner + std::min(2LL, bead_count) * optimal_width_outer;
}

coord_t CenterDeviationBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    if (lower_bead_count % 2 == 0)
    { // when we add the extra bead in the middle
        return this->getOptimalThickness(lower_bead_count) + underfill_bound;
    }
    else
    { // when we move away from the strategy which replaces two beads by a single one in the middle
        return this->getOptimalThickness(lower_bead_count + 1) - overfill_bound;
    }
}

coord_t CenterDeviationBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    coord_t thickness_left = thickness;
    coord_t naive_count = 0;
    naive_count += std::min(2LL, ((thickness / 2 + optimal_width_outer / 2) / optimal_width_outer) * 2);
    thickness_left -= naive_count * optimal_width_outer;
    if (thickness_left >= (optimal_width_inner / 2))
    {
        naive_count += ((thickness_left / 2 + optimal_width_inner / 2) / optimal_width_inner) * 2;
    }

    const coord_t overfill = this->getOptimalThickness(naive_count) - thickness;
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


} // namespace cura
