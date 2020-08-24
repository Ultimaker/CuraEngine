//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cassert>

#include "LimitedBeadingStrategy.h"

namespace cura
{

LimitedBeadingStrategy::Beading LimitedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (bead_count <= max_bead_count)
    {
        return parent->compute(thickness, bead_count);
    }
    assert(bead_count == max_bead_count + 1);

    coord_t optimal_thickness = parent->getOptimalThickness(max_bead_count);
    Beading ret = parent->compute(optimal_thickness, max_bead_count);
    ret.left_over += thickness - ret.total_thickness;
    ret.total_thickness = thickness;
    
    // Enforece symmetry
    if (bead_count % 2 == 1)
    {
        ret.toolpath_locations[bead_count / 2] = thickness / 2;
        ret.bead_widths[bead_count / 2] = thickness - optimal_thickness;
    }
    for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
    {
        ret.toolpath_locations[bead_count - 1 - bead_idx] = thickness - ret.toolpath_locations[bead_idx];
    }
    return ret;
}

coord_t LimitedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    if (bead_count <= max_bead_count)
    {
        return parent->getOptimalThickness(bead_count);
    }
    return 10000000; // 10 meter
}

coord_t LimitedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < max_bead_count)
    {
        return parent->getTransitionThickness(lower_bead_count);
    }
    if (lower_bead_count == max_bead_count)
    {
        return parent->getOptimalThickness(lower_bead_count + 1) - 10;
    }
    return 9000000; // 9 meter
}

coord_t LimitedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    coord_t parent_bead_count = parent->getOptimalBeadCount(thickness);
    if (parent_bead_count <= max_bead_count)
    {
        return parent->getOptimalBeadCount(thickness);
    }
    else if (parent_bead_count == max_bead_count + 1)
    {
        if (thickness < parent->getOptimalThickness(max_bead_count + 1) - 10)
            return max_bead_count;
        else 
            return max_bead_count + 1;
    }
    else return max_bead_count + 1;
}

} // namespace cura
