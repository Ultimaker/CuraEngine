//Copyright (c) 2019 Ultimaker B.V.


#include "LimitedDistributedBeadingStrategy.h"

namespace arachne
{

LimitedDistributedBeadingStrategy::Beading LimitedDistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (thickness < DistributedBeadingStrategy::transition_thickness(max_bead_count))
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

coord_t LimitedDistributedBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    if (bead_count <= max_bead_count)
    {
        return DistributedBeadingStrategy::optimal_thickness(bead_count);
    }
    return 10000000; // 10 meter
}

coord_t LimitedDistributedBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < max_bead_count)
    {
        return DistributedBeadingStrategy::transition_thickness(lower_bead_count);
    }
    return 9000000; // 9 meter
}

coord_t LimitedDistributedBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    if (thickness < DistributedBeadingStrategy::optimal_thickness(max_bead_count))
    {
        return DistributedBeadingStrategy::optimal_bead_count(thickness);
    }
    return max_bead_count;
}

} // namespace arachne
