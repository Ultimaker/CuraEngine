//Copyright (c) 2020 Ultimaker B.V.

#include <cassert>

#include "BeadingStrategy.h"

namespace arachne
{

bool BeadingStrategy::checkTranisionThicknessConsistency(const BeadingStrategy* strategy)
{
    // LA-TODO: can this work check
    return false;
}

coord_t BeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    if (lower_bead_count == 0)
    {
        return 10;
    }
    return default_transition_length;
}

float BeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    coord_t lower_optimum = optimal_thickness(lower_bead_count);
    coord_t transition_point = transition_thickness(lower_bead_count);
    coord_t upper_optimum = optimal_thickness(lower_bead_count + 1);
    return 1.0 - float(transition_point - lower_optimum) / float(upper_optimum - lower_optimum);
}

/*virtual*/ std::vector<coord_t> BeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
{
    return std::vector<coord_t>();
}

} // namespace arachne
