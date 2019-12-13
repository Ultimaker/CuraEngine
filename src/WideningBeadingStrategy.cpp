//Copyright (c) 2019 Ultimaker B.V.


#include "WideningBeadingStrategy.h"

namespace arachne
{

WideningBeadingStrategy::Beading WideningBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (thickness < optimal_width)
    {
        Beading ret;
        ret.total_thickness = thickness;
        if (thickness >= min_input_width)
        {
            ret.bead_widths.emplace_back(std::max(thickness, min_output_width));
            ret.toolpath_locations.emplace_back(thickness / 2);
        }
        else
        {
            ret.left_over = thickness;
        }
        return ret;
    }
    else
    {
        return parent->compute(thickness, bead_count);
    }
}

coord_t WideningBeadingStrategy::optimal_thickness(coord_t bead_count) const
{
    return parent->optimal_thickness(bead_count);
}

coord_t WideningBeadingStrategy::transition_thickness(coord_t lower_bead_count) const
{
    if (lower_bead_count == 0)
    {
        return min_input_width;
    }
    else
    {
        return parent->transition_thickness(lower_bead_count);
    }
}

coord_t WideningBeadingStrategy::optimal_bead_count(coord_t thickness) const
{
    if (thickness < min_input_width) return 0;
    coord_t ret = parent->optimal_bead_count(thickness);
    if (thickness >= min_input_width && ret < 1) return 1;
    return ret;
}

coord_t WideningBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const 
{
    return parent->getTransitioningLength(lower_bead_count);
}

float WideningBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    return parent->getTransitionAnchorPos(lower_bead_count);
}

std::vector<coord_t> WideningBeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
{
    std::vector<coord_t> ret;
    ret.emplace_back(min_output_width);
    std::vector<coord_t> pret = parent->getNonlinearThicknesses(lower_bead_count);
    ret.insert(ret.end(), pret.begin(), pret.end());
    return ret;
}

} // namespace arachne
