//Copyright (c) 2019 Ultimaker B.V.


#ifndef SINGLE_BEAD_BEADING_STRATEGY_H
#define SINGLE_BEAD_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{
    using namespace cura;

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class SingleBeadBeadingStrategy : public BeadingStrategy
{
public:
    SingleBeadBeadingStrategy(const coord_t bead_width, float transitioning_angle = M_PI / 4)
    : BeadingStrategy(bead_width, transitioning_angle)
    {
        name = "SingleBeadBeadingStrategy";
    }
    virtual ~SingleBeadBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
    coord_t getTransitioningLength(coord_t lower_bead_count) const override;
};




} // namespace arachne
#endif // SINGLE_BEAD_BEADING_STRATEGY_H
