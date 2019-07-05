//Copyright (c) 2019 Ultimaker B.V.


#ifndef CONSTANT_BEADING_STRATEGY_H
#define CONSTANT_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class ConstantBeadingStrategy : public BeadingStrategy
{
public:
    const coord_t bead_count;
    ConstantBeadingStrategy(const coord_t bead_width, const coord_t bead_count)
    : BeadingStrategy(bead_width)
    , bead_count(bead_count)
    {
    }
    Beading compute(coord_t thickness, coord_t bead_count) const;
    coord_t optimal_thickness(coord_t bead_count) const;
    coord_t transition_thickness(coord_t lower_bead_count) const;
    coord_t optimal_bead_count(coord_t thickness) const;
};




} // namespace arachne
#endif // CONSTANT_BEADING_STRATEGY_H
