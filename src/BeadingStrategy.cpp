//Copyright (c) 2019 Ultimaker B.V.

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

} // namespace arachne
