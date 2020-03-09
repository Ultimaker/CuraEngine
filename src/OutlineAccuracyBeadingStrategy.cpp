//Copyright (c) 2019 Ultimaker B.V.


#include "OutlineAccuracyBeadingStrategy.h"

namespace arachne
{

OutlineAccuracyBeadingStrategy::Beading OutlineAccuracyBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        ret.bead_widths.resize(bead_count);
        if (bead_count == 1)
        {
            ret.bead_widths[0] = thickness;
        }
        else if (bead_count > 2 && thickness >= 2 * optimal_outer_width + (bead_count - 2) * min_width)
        { // The outer width is able to be the optimal
            ret.bead_widths[0] = optimal_outer_width;
            for (coord_t bead_idx = 1; bead_idx < (bead_count + 1) / 2; bead_idx++)
            {
                ret.bead_widths[bead_idx] = (thickness - 2 * optimal_outer_width) / (bead_count - 2);
            }
        }
        else if (thickness >= bead_count * min_width)
        {
            ret.bead_widths[0] = (thickness - min_width * (bead_count - 2)) / 2;
            for (coord_t bead_idx = 1; bead_idx < (bead_count + 1) / 2; bead_idx++)
            {
                ret.bead_widths[bead_idx] = min_width;
            }
        }
        else
        { // Shouldn't happen too often hopefully! widths are neccesarily smaller then the min_width!
            for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
            {
                ret.bead_widths[bead_idx] = thickness / bead_count;
            }
        }
        // Derive toolpath_locations
        ret.toolpath_locations.resize(bead_count);
        for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
        {
            ret.toolpath_locations[bead_idx] = ret.bead_widths[bead_idx] / 2;
            for (coord_t lower_bead_idx = 0; lower_bead_idx < bead_idx; lower_bead_idx++)
            {
                ret.toolpath_locations[bead_idx] += ret.bead_widths[lower_bead_idx];
            }
        }

        if (bead_count % 2 == 1)
        {
            ret.toolpath_locations[bead_count / 2] = thickness / 2;
        }
        // Apply symmetry
        for (coord_t bead_idx = (bead_count + 1) / 2; bead_idx < bead_count; bead_idx++)
        {
            ret.bead_widths[bead_idx] = ret.bead_widths[bead_count - 1 - bead_idx];
            ret.toolpath_locations[bead_idx] = thickness - ret.toolpath_locations[bead_count - 1 - bead_idx];
        }
        
        for (coord_t w : ret.bead_widths) 
        {
            assert(w != 0);
        }
        for (coord_t l : ret.toolpath_locations) 
        {
            assert(l != 0);
        }
        
        ret.left_over = 0;
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t OutlineAccuracyBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    if (bead_count <= 0) 
    {
        return 0;
    }
    if (bead_count <= 2) 
    {
        return bead_count * optimal_outer_width;
    }
    return optimal_outer_width * 2 + (bead_count - 2) * optimal_width;
}

coord_t OutlineAccuracyBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count == 0)
    {
        return min_width;
    }
    if (lower_bead_count == 1) 
    {
        return 2 * min_width;
    }
    
    coord_t normal_transition_thickness = optimal_width * (lower_bead_count - 2) + 2 * optimal_outer_width + optimal_width / 2;
    if (lower_bead_count == 2)
    {
        return std::max(normal_transition_thickness, 3 * min_width);
    }
    return normal_transition_thickness;
}

coord_t OutlineAccuracyBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    if (thickness < min_width)
    {
        return 0;
    }
    if (thickness < 2 * min_width)
    {
        return 1;
    }
    if (thickness < 3 * min_width)
    { 
        return 2;
    }
    return 2 + (thickness - 2 * optimal_outer_width + optimal_width / 2) / optimal_width;
}


} // namespace arachne
