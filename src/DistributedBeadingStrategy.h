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
    DistributedBeadingStrategy(const coord_t min_width, const coord_t optimal_width, const coord_t max_width)
    : BeadingStrategy(optimal_width)
    , min_width(min_width)
    , max_width(max_width)
    {
    }
    Beading compute(coord_t thickness, coord_t bead_count) const;
    coord_t optimal_thickness(coord_t bead_count) const;
    coord_t transition_thickness(coord_t lower_bead_count) const;
    coord_t optimal_bead_count(coord_t thickness) const;
};




} // namespace arachne
#endif // DISTRIBUTED_BEADING_STRATEGY_H
