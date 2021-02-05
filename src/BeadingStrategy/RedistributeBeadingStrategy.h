//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H
#define REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

#include "../settings/types/Ratio.h"

namespace cura
{
    /*!
     * Made a meta-beading-strat. that takes outer and inner wall widths into account. `
     *
     * The idea is as follows: Initially, for the parent (base) beading strategy, set the 'optimum' width as a weighted average defined by
     * 'what the total thickness of the fill would be if max-bead width is always reached' (so take the situation where the actually
     * requested total wall width is reached as the ground truth we base all the rest on). The base beading-strategy will thus produce the
     * 'right' number of walls but the wrong bead-widths (and locations) -- because it assumes the walls are some average. This is then
     * rectified by the newly introduced meta-strategy, which redistributes the bead-widths. It takes a look at the factor of the thickness
     * the outer and inner walls would have occupied in the ideal situation and applies that to the current situation. This keeps a) the
     * original beadings simple to understand and debug b) the meanings and behaviour of the original beadings intact while still
     * c) preserving the widths of the outer and inner walls the user (settings) has/have input at the very least in the case of 'full'
     * walls (and does a decent attempt at at least preserving the relative relation between those two values with a lesser thickness).
     */
    class RedistributeBeadingStrategy : public BeadingStrategy
    {
    public:
        /*!
         * /param optimal_width_outer Outer wall width, guaranteed to be the actual (save rounding errors) at a bead count if the parent
         *                            strategies' optimum bead width is a weighted average of the outer and inner walls at that bead count.
         * /param optimal_width_outer Inner wall width, guaranteed to be the actual (save rounding errors) at a bead count if the parent
         *                            strategies' optimum bead width is a weighted average of the outer and inner walls at that bead count.
         * /param outer_wall_lock_factor How much the outer walls should be forced to their optimal width.
         *                               From '1.0': always, unless impossible, to '0.0': ignore the lock factor, redistribute as normal.
         */
        RedistributeBeadingStrategy(const coord_t optimal_width_outer, const coord_t optimal_width_inner, const Ratio outer_wall_lock_factor, BeadingStrategy* parent) :
            BeadingStrategy(parent->optimal_width, parent->default_transition_length, parent->transitioning_angle),
            parent(parent),
            optimal_width_outer(optimal_width_outer),
            optimal_width_inner(optimal_width_inner),
            outer_wall_lock_factor(outer_wall_lock_factor),
            outer_wall_lock_inverse(1.0_r - outer_wall_lock_factor)
        {
            name = "RedistributeBeadingStrategy";
        }

        virtual ~RedistributeBeadingStrategy() = default;

        Beading compute(coord_t thickness, coord_t bead_count) const override;

        coord_t getOptimalThickness(coord_t bead_count) const override { return parent->getOptimalThickness(bead_count);  }
        coord_t getTransitionThickness(coord_t lower_bead_count) const override { return parent->getTransitionThickness(lower_bead_count); }
        coord_t getOptimalBeadCount(coord_t thickness) const override { return parent->getOptimalBeadCount(thickness); }
        coord_t getTransitioningLength(coord_t lower_bead_count) const override { return parent->getTransitioningLength(lower_bead_count); }
        float getTransitionAnchorPos(coord_t lower_bead_count) const override { return parent->getTransitionAnchorPos(lower_bead_count); }

        virtual std::string toString() const { return std::string("RedistributeBeadingStrategy+") + parent->toString(); }

    private:
        BeadingStrategy* parent;
        coord_t optimal_width_outer;
        coord_t optimal_width_inner;

        Ratio outer_wall_lock_factor;
        Ratio outer_wall_lock_inverse;
    };

} // namespace cura
#endif // INWARD_DISTRIBUTED_BEADING_STRATEGY_H
