//Copyright (c) 2019 Ultimaker B.V.


#ifndef OUTLINE_ACCURACY_BEADING_STRATEGY_H
#define OUTLINE_ACCURACY_BEADING_STRATEGY_H

#include <cassert>

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class OutlineAccuracyBeadingStrategy : public BeadingStrategy
{
    const coord_t optimal_outer_width;
    const coord_t min_width;
public:
    OutlineAccuracyBeadingStrategy(const coord_t optimal_width, coord_t default_transition_length, const coord_t optimal_outer_width, const coord_t min_width, float transitioning_angle)
    : BeadingStrategy(optimal_width, default_transition_length, transitioning_angle)
    , optimal_outer_width(optimal_outer_width)
    , min_width(min_width)
    {
        assert(min_width < optimal_outer_width);
        assert(optimal_outer_width < optimal_width);
    }
    virtual ~OutlineAccuracyBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    virtual std::string toString() const override { return "OutlineAccuracyBeadingStrategy";}
};




} // namespace arachne
#endif // OUTLINE_ACCURACY_BEADING_STRATEGY_H
