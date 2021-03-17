//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H
#define REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

#include "../settings/types/Ratio.h"

namespace cura
{
    /*!
     * A meta-beading-strategy that takes outer and inner wall widths into account.
     *
     * The outer wall will try to keep a constant width by only applying the beading strategy on the inner walls. This
     * ensures that this outer wall doesn't react to changes happening to inner walls. It will limit print artifacts on
     * the surface of the print. Although this strategy technically deviates from the original philosophy of the paper.
     * It will generally results in better prints because of a smoother motion and less variation in extrusion width in
     * the outer walls.
     *
     * If the thickness of the model is less then two times the optimal outer wall width and once the minimum inner wall
     * width it will keep the minimum inner wall at a minimum constant and vary the outer wall widths symmetrical. Until
     * The thickness of the model is that of at least twice the optimal outer wall width it will then use two
     * symmetrical outer walls only. Until it transitions into a single outer wall. These last scenario's are always
     * symmetrical in nature, disregarding the user specified strategy.
     */
    class RedistributeBeadingStrategy : public BeadingStrategy
    {
    public:
        /*!
         * /param optimal_width_outer         Outer wall width, guaranteed to be the actual (save rounding errors) at a
         *                                    bead count if the parent strategies' optimum bead width is a weighted
         *                                    average of the outer and inner walls at that bead count.
         * /param optimal_width_outer         Inner wall width, guaranteed to be the actual (save rounding errors) at a
         *                                    bead count if the parent strategies' optimum bead width is a weighted
         *                                    average of the outer and inner walls at that bead count.
         * /param minimum_variable_line_width Minimum factor that the variable line might deviate from the optimal width.
         */
        RedistributeBeadingStrategy(const coord_t optimal_width_outer,
                                    const coord_t optimal_width_inner,
                                    const double minimum_variable_line_width,
                                    BeadingStrategyPtr parent);

        virtual ~RedistributeBeadingStrategy() override = default;

        Beading compute(coord_t thickness, coord_t bead_count) const override;

        coord_t getOptimalThickness(coord_t bead_count) const override;
        coord_t getTransitionThickness(coord_t lower_bead_count) const override;
        coord_t getOptimalBeadCount(coord_t thickness) const override;
        coord_t getTransitioningLength(coord_t lower_bead_count) const override;
        float getTransitionAnchorPos(coord_t lower_bead_count) const override;

        virtual std::string toString() const;

    protected:
        /*!
         * Determine the outer bead width.
         *
         * According to the following logic:
         *  - If the thickness of the model is more then twice the optimal outer bead width and the minimum inner bead
         *  width it will return the optimal outer bead width.
         *  - If the thickness is less then twice the optimal outer bead width and the minimum inner bead width, but
         *  more them twice the optimal outer bead with it will return the optimal bead width minus half the inner bead
         *  width.
         *  - If the thickness is less then twice the optimal  outer bead width it will return half the thickness as
         *  outer bead width
         *
         * \param thickness Thickness of the total beads.
         * \param optimal_width_outer User specified optimal outer bead width.
         * \param minimum_width_inner Inner bead width times the minimum variable line width.
         * \return The outer bead width.
         */
        static coord_t getOptimalOuterBeadWidth(coord_t thickness, coord_t optimal_width_outer, coord_t minimum_width_inner);

        /*!
         * Moves the beads towards the outer edges of thickness and ensures that the outer walls are locked in location
         * \param beading The beading instance.
         * \param thickness The thickness of the bead.
         */
        static void resetToolPathLocations(Beading& beading, coord_t thickness);

        /*!
         * Filters and validates the beads, to ensure that all inner beads are at least the minimum bead width.
         *
         * \param beading The beading instance.
         * \param minimum_width_inner Inner bead width times the minimum variable line width.
         * \return true if beads are removed.
         */
        static bool validateInnerBeadWidths(Beading& beading, coord_t minimum_width_inner);

        BeadingStrategyPtr parent;
        coord_t optimal_width_outer;
        coord_t optimal_width_inner;
        double minimum_variable_line_width;
    };

} // namespace cura
#endif // INWARD_DISTRIBUTED_BEADING_STRATEGY_H
