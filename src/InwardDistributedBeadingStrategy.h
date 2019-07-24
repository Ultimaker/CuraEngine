//Copyright (c) 2019 Ultimaker B.V.


#ifndef INWARD_DISTRIBUTED_BEADING_STRATEGY_H
#define INWARD_DISTRIBUTED_BEADING_STRATEGY_H

#include "DistributedBeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which divides the discrepancy between the current thickness and optimal thickness mainly to the inner beads.
 * This causes the outer inset to be constant width almost everywhere.
 */
class InwardDistributedBeadingStrategy : public DistributedBeadingStrategy
{
public:
    InwardDistributedBeadingStrategy(const coord_t optimal_width, float transitioning_angle)
    : DistributedBeadingStrategy(optimal_width, transitioning_angle)
    {
    }
    Beading compute(coord_t thickness, coord_t bead_count) const override;
};




} // namespace arachne
#endif // INWARD_DISTRIBUTED_BEADING_STRATEGY_H
