//Copyright (c) 2019 Ultimaker B.V.


#include "WideningBeadingStrategy.h"

namespace arachne
{

WideningBeadingStrategy::Beading WideningBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret = parent->compute(thickness, bead_count);
    if (ret.bead_widths.size() == 1)
    {
        ret.bead_widths[0] = std::max(ret.bead_widths[0], min_output_width);
    }
    return ret;
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
    coord_t ret = parent->optimal_bead_count(thickness);
    if (thickness > min_input_width && ret < 1) return 1;
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


} // namespace arachne
