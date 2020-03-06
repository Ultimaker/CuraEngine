//Copyright (c) 2020 Ultimaker B.V.

#include <cassert>

#include "BeadingStrategy.h"

namespace arachne
{

bool BeadingStrategy::checkTranisionThicknessConsistency(const BeadingStrategy* strategy)
{
    bool ret = false;
#ifdef DEBUG
    std::vector<coord_t> transition_thicknesses;
    coord_t prev_bead_count = 0;
    for (coord_t thickness = 0; thickness < strategy->optimal_width * 10; thickness++)
    {
        coord_t optimal_bead_count = strategy->optimal_bead_count(thickness);
        if (optimal_bead_count != prev_bead_count)
        {
            transition_thicknesses.emplace_back(thickness);
            prev_bead_count = optimal_bead_count;
        }
    }
    for (size_t transition_idx = 0; transition_idx < transition_thicknesses.size(); transition_idx++)
    {
        coord_t calculated_thickness = transition_thicknesses[transition_idx];
        coord_t supposed_thickness = strategy->transition_thickness(transition_idx);
        assert(std::abs(calculated_thickness - supposed_thickness) < 10);
        if (std::abs(calculated_thickness - supposed_thickness) >= 10)
            ret = true;
    }
#endif // DEBUG
    return ret;
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

virtual std::vector<coord_t> BeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
{
    return std::vector<coord_t>();
}

} // namespace arachne
