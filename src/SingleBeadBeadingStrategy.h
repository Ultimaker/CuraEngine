//Copyright (c) 2019 Ultimaker B.V.


#ifndef SINGLE_BEAD_BEADING_STRATEGY_H
#define SINGLE_BEAD_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class SingleBeadBeadingStrategy : public BeadingStrategy
{
public:
    SingleBeadBeadingStrategy(const coord_t bead_width, float transitioning_angle = M_PI / 4)
    : BeadingStrategy(bead_width, transitioning_angle)
    {
    }
    virtual ~SingleBeadBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    coord_t getTransitioningLength(coord_t lower_bead_count) const override;
    virtual std::string toString() const override { return "SingleBeadBeadingStrategy";}
};




} // namespace arachne
#endif // SINGLE_BEAD_BEADING_STRATEGY_H
