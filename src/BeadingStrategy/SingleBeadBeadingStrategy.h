//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SINGLE_BEAD_BEADING_STRATEGY_H
#define SINGLE_BEAD_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{
    using namespace cura;

/*!
 * This beading strategy keeps all lines at their optimal width, causing gaps to
 * appear if the part is too wide or too narrow. Except if the part is thinner
 * than a single line. Then the line is reduced in width.
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
