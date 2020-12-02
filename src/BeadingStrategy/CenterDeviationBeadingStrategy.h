//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef CENTER_DEVIATION_BEADING_STRATEGY_H
#define CENTER_DEVIATION_BEADING_STRATEGY_H

#include "../settings/types/Ratio.h" //For the wall transition threshold.
#include "BeadingStrategy.h"

namespace cura
{

/*!
 * This beading strategy makes the deviation in the thickness of the part
 * entirely compensated by the innermost wall.
 *
 * The outermost walls all use the ideal width, as far as possible.
 */
class CenterDeviationBeadingStrategy : public BeadingStrategy
{
    coord_t overfill_bound; // Amount of overfill before the two innermost beads are replaced by a single in the middle.
    coord_t underfill_bound; // Amount of underfil before a single bead in the middle is placed
public:
    CenterDeviationBeadingStrategy(const coord_t pref_bead_width, const AngleRadians transitioning_angle, const Ratio wall_transition_threshold)
    : BeadingStrategy(pref_bead_width, pref_bead_width / 2, transitioning_angle)
    , overfill_bound(pref_bead_width * (1.0f - wall_transition_threshold))
    , underfill_bound(pref_bead_width * (1.0f - (wall_transition_threshold * 0.95f)))
    {
        name = "CenterDeviationBeadingStrategy";
    }
    virtual ~CenterDeviationBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace cura
#endif // CENTER_DEVIATION_BEADING_STRATEGY_H
