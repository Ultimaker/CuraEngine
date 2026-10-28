// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "beading_strategy/BeadingStrategy.h"
#include "types/Ratio.h" // For the wall transition threshold.

namespace curaengine::beading_strategy
{

/*!
 * This beading strategy chooses a wall count that would make the line width
 * deviate the least from the optimal line width, and then distributes the lines
 * evenly among the thickness available.
 */
class DistributedBeadingStrategy : public BeadingStrategy
{
protected:
    double one_over_distribution_radius_squared_; // (1 / distribution_radius)^2

public:
    /*!
     * \param distribution_radius the radius (in number of beads) over which to distribute the discrepancy between the feature size and the optimal thickness
     */
    DistributedBeadingStrategy(
        const utils::coord_t optimal_width,
        const utils::coord_t default_transition_length,
        const types::AngleRadians transitioning_angle,
        const types::Ratio wall_split_middle_threshold,
        const types::Ratio wall_add_middle_threshold,
        const int distribution_radius);

    virtual ~DistributedBeadingStrategy() override
    {
    }

    Beading compute(utils::coord_t thickness, utils::coord_t bead_count) const override;
    utils::coord_t getOptimalBeadCount(utils::coord_t thickness) const override;
};

} // namespace curaengine::beading_strategy
