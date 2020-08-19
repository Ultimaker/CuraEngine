//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LimitedDistributedBeadingStrategy.h"

namespace cura
{

LimitedDistributedBeadingStrategy::Beading LimitedDistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (thickness < DistributedBeadingStrategy::getTransitionThickness(max_bead_count))
    {
        return DistributedBeadingStrategy::compute(thickness, bead_count);
    }

    Beading ret;
    ret.total_thickness = thickness;
    ret.left_over = thickness - max_bead_count * optimal_width;
    ret.bead_widths.resize(max_bead_count, optimal_width);
    if (max_bead_count % 2 == 1)
    {
        ret.bead_widths[max_bead_count / 2] = thickness - (max_bead_count - 1) * optimal_width;
    }
    for (int i = 0; i < max_bead_count / 2; i++)
    {
        ret.toolpath_locations.emplace_back(optimal_width * i + optimal_width / 2);
    }
    if (max_bead_count % 2 == 1)
    {
        ret.toolpath_locations.emplace_back(thickness / 2);
    }
    for (int i = max_bead_count / 2 - 1; i >= 0; i--)
    {
        ret.toolpath_locations.emplace_back(thickness - optimal_width * i - optimal_width / 2);
    }
    return ret;
}

coord_t LimitedDistributedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    if (bead_count <= max_bead_count)
    {
        return DistributedBeadingStrategy::getOptimalThickness(bead_count);
    }
    return 10000000; // 10 meter
}

coord_t LimitedDistributedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < max_bead_count)
    {
        return DistributedBeadingStrategy::getTransitionThickness(lower_bead_count);
    }
    return 9000000; // 9 meter
}

coord_t LimitedDistributedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    if (thickness < DistributedBeadingStrategy::getOptimalThickness(max_bead_count))
    {
        return DistributedBeadingStrategy::getOptimalBeadCount(thickness);
    }
    return max_bead_count;
}

} // namespace cura
