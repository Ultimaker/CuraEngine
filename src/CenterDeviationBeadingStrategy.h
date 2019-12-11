//Copyright (c) 2019 Ultimaker B.V.


#ifndef CENTER_DEVIATION_BEADING_STRATEGY_H
#define CENTER_DEVIATION_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class CenterDeviationBeadingStrategy : public BeadingStrategy
{
    coord_t overfill_bound; // amount of overfill before the two innermost beads are replaced by a single in the middle.
    coord_t underfill_bound; // amount of underfil before a single bead in the middle is placed
public:
    CenterDeviationBeadingStrategy(const coord_t pref_bead_width, float transitioning_angle, float min_diameter = 0.8, float max_diameter = 1.25)
    : BeadingStrategy(pref_bead_width, transitioning_angle)
    , overfill_bound(pref_bead_width - min_diameter * pref_bead_width)
    , underfill_bound((max_diameter - 1) * pref_bead_width * 2)
    {
    }
    virtual ~CenterDeviationBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    coord_t getTransitioningLength(coord_t lower_bead_count) const override
    {
        return optimal_width / 2;
    }
    virtual std::string toString() const override { return "CenterDeviationBeadingStrategy";}
};




} // namespace arachne
#endif // CENTER_DEVIATION_BEADING_STRATEGY_H
