//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef INWARD_DISTRIBUTED_BEADING_STRATEGY_H
#define INWARD_DISTRIBUTED_BEADING_STRATEGY_H

#include "DistributedBeadingStrategy.h"

namespace arachne
{
    using namespace cura;

/*!
 * A beading strategy which divides the discrepancy between the current and
 * optimal thickness mainly to the inner beads.
 * 
 * The number of countours is optimised to reduce the discrepancy as much as
 * possible.
 * 
 * This causes the outer inset to be constant width almost everywhere.
 */
class InwardDistributedBeadingStrategy : public DistributedBeadingStrategy
{
public:
    /*!
     * \param distribution_radius the radius (in number of beads) over which to distribute the discrepancy between the feature size and the optimal thickness
     */
    InwardDistributedBeadingStrategy(const coord_t optimal_width, coord_t default_transition_length, float transitioning_angle, float distribution_radius)
    : DistributedBeadingStrategy(optimal_width, default_transition_length, transitioning_angle)
    , one_over_distribution_radius_squared(1.0f / distribution_radius * 1.0f / distribution_radius)
    {
        name = "InwardDistributedBeadingStrategy";
    }
    virtual ~InwardDistributedBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
private:
    float one_over_distribution_radius_squared; // (1 / distribution_radius)^2
};




} // namespace arachne
#endif // INWARD_DISTRIBUTED_BEADING_STRATEGY_H
