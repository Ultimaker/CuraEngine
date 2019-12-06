//Copyright (c) 2019 Ultimaker B.V.


#ifndef NAIVE_BEADING_STRATEGY_H
#define NAIVE_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class NaiveBeadingStrategy : public BeadingStrategy
{
public:
    NaiveBeadingStrategy(const coord_t bead_width)
    : BeadingStrategy(bead_width, 0)
    {
    }
    virtual ~NaiveBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    virtual std::string toString() const override { return "NaiveBeadingStrategy";}
};




} // namespace arachne
#endif // NAIVE_BEADING_STRATEGY_H
