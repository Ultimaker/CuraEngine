// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DISTRIBUTED_BEADING_STRATEGY_H
#define DISTRIBUTED_BEADING_STRATEGY_H

#include "../settings/types/Ratio.h" // For the wall transition threshold.
#include "BeadingStrategy.h"

namespace cura
{

/*!
 * This beading strategy chooses a wall count that would make the line width
 * deviate the least from the optimal line width, and then distributes the lines
 * evenly among the thickness available.
 */
class DistributedBeadingStrategy : public BeadingStrategy
{
protected:
     // For uneven numbers of lines: Minimum factor of the optimal width for which the middle line will be split into two lines.
    Ratio wall_split_middle_threshold;
    
    // For even numbers of lines: Minimum factor of the optimal width for which a new middle line will be added between the two innermost lines.
    Ratio wall_add_middle_threshold;

    float one_over_distribution_radius_squared; // (1 / distribution_radius)^2

public:
    /*!
    * \param distribution_radius the radius (in number of beads) over which to distribute the discrepancy between the feature size and the optimal thickness
    */
    DistributedBeadingStrategy( const coord_t optimal_width,
                                const coord_t default_transition_length,
                                const AngleRadians transitioning_angle,
                                const Ratio wall_split_middle_threshold,
                                const Ratio wall_add_middle_threshold,
                                const int distribution_radius);

    virtual ~DistributedBeadingStrategy() override {}

    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace cura
#endif // DISTRIBUTED_BEADING_STRATEGY_H
