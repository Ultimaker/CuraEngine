//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef NAIVE_BEADING_STRATEGY_H
#define NAIVE_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace cura
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class NaiveBeadingStrategy : public BeadingStrategy
{
public:
    NaiveBeadingStrategy(const coord_t bead_width)
    : BeadingStrategy(bead_width, /*default_transition_length=*/ 10, 0)
    {
    }
    virtual ~NaiveBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
    virtual std::string toString() const override { return "NaiveBeadingStrategy";}
};




} // namespace cura
#endif // NAIVE_BEADING_STRATEGY_H
