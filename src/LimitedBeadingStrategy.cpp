//Copyright (c) 2019 Ultimaker B.V.


#include "LimitedBeadingStrategy.h"

namespace arachne
{

LimitedBeadingStrategy::Beading LimitedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (thickness < parent->transition_thickness(max_bead_count))
    {
        return parent->compute(thickness, bead_count);
    }

    Beading ret = parent->compute(parent->optimal_thickness(bead_count), bead_count);
    ret.left_over += thickness - ret.total_thickness;
    ret.total_thickness = thickness;
    
    // enforece symmetry
    if (bead_count % 2 == 1)
        ret.toolpath_locations[bead_count / 2] = thickness / 2;
    for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
    {
        ret.toolpath_locations[bead_count - 1 - bead_idx] = thickness - ret.toolpath_locations[bead_count];
    }
    return ret;
}

coord_t LimitedBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    if (bead_count <= max_bead_count)
    {
        return parent->optimal_thickness(bead_count);
    }
    return 10000000; // 10 meter
}

coord_t LimitedBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < max_bead_count)
    {
        return parent->transition_thickness(lower_bead_count);
    }
    return 9000000; // 9 meter
}

coord_t LimitedBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    if (thickness < parent->optimal_thickness(max_bead_count))
    {
        return parent->optimal_bead_count(thickness);
    }
    return max_bead_count;
}

std::vector<coord_t> LimitedBeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
{
    std::vector<coord_t> ret = parent->getNonlinearThicknesses(lower_bead_count);
    if (lower_bead_count == max_bead_count)
    {
        ret.emplace_back(optimal_thickness(max_bead_count));
        ret.emplace_back(parent->transition_thickness(max_bead_count) - 10); // -10 to fix rounding bug
        ret.emplace_back(parent->transition_thickness(max_bead_count) + optimal_width / 2); // ideally the transition from a thin section to a section where all beads have preferred widths is a constant length, while now the length is determined by the slope of the marked region.
    }
    return ret;
}
} // namespace arachne
