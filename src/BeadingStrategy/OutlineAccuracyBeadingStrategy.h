//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef OUTLINE_ACCURACY_BEADING_STRATEGY_H
#define OUTLINE_ACCURACY_BEADING_STRATEGY_H

#include <cassert>

#include "BeadingStrategy.h"

namespace arachne
{
    using namespace cura;

/*!
 * This beading strategy makes the outer wall have the optimal width if possible
 * (but a lower width if necessary) and evenly divides the inner walls among the
 * remaining thickness available.
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
        name = "OutlineAccuracyBeadingStrategy";
    }
    
    virtual ~OutlineAccuracyBeadingStrategy() override
    {}
    
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace arachne
#endif // OUTLINE_ACCURACY_BEADING_STRATEGY_H
