//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DISTRIBUTED_BEADING_STRATEGY_H
#define DISTRIBUTED_BEADING_STRATEGY_H

#include "../settings/types/Ratio.h" //For the wall transition threshold.
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
private:
    /*!
     * Threshold line width at which it will use fewer, wider lines instead of
     * reducing the line width further.
     */
    Ratio wall_transition_threshold;

public:
    DistributedBeadingStrategy(const coord_t optimal_width, const coord_t default_transition_length, const AngleRadians transitioning_angle, const Ratio wall_transition_threshold)
    : BeadingStrategy(optimal_width, default_transition_length, transitioning_angle)
    , wall_transition_threshold(wall_transition_threshold)
    {
        name = "DistributedBeadingStrategy";
    }
    virtual ~DistributedBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace cura
#endif // DISTRIBUTED_BEADING_STRATEGY_H
