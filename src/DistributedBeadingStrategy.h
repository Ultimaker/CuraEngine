//Copyright (c) 2019 Ultimaker B.V.


#ifndef DISTRIBUTED_BEADING_STRATEGY_H
#define DISTRIBUTED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class DistributedBeadingStrategy : public BeadingStrategy
{
public:
    const coord_t min_width;
    const coord_t max_width;
    DistributedBeadingStrategy(const coord_t min_width, const coord_t optimal_width, const coord_t max_width, float transitioning_angle)
    : BeadingStrategy(optimal_width, transitioning_angle)
    , min_width(min_width)
    , max_width(max_width)
    {
    }
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    coord_t getTransitioningLength(coord_t dR, coord_t dd) const override;
};




} // namespace arachne
#endif // DISTRIBUTED_BEADING_STRATEGY_H
