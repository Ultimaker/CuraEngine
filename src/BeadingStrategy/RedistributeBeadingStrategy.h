//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H
#define REDISTRIBUTE_DISTRIBUTED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace cura
{
    /*!
     */
    class RedistributeBeadingStrategy : public BeadingStrategy
    {
    public:
        /*!
         */
        RedistributeBeadingStrategy(coord_t optimal_width_outer, coord_t optimal_width_inner, BeadingStrategy* parent) :
            BeadingStrategy(parent->optimal_width, parent->default_transition_length, parent->transitioning_angle),
            parent(parent),
            optimal_width_outer(optimal_width_outer),
            optimal_width_inner(optimal_width_inner)
        {
            name = "RedistributeBeadingStrategy";
        }

        virtual ~RedistributeBeadingStrategy() override {}

        Beading compute(coord_t thickness, coord_t bead_count) const override;

        coord_t getOptimalThickness(coord_t bead_count) const override { return parent->getOptimalThickness(bead_count);  }
        coord_t getTransitionThickness(coord_t lower_bead_count) const override { return parent->getTransitionThickness(lower_bead_count); }
        coord_t getOptimalBeadCount(coord_t thickness) const override { return parent->getOptimalBeadCount(thickness); }
        coord_t getTransitioningLength(coord_t lower_bead_count) const override { return parent->getTransitioningLength(lower_bead_count); }
        float getTransitionAnchorPos(coord_t lower_bead_count) const override { return parent->getTransitionAnchorPos(lower_bead_count); }

        virtual std::string toString() const override { return std::string("RedistributeBeadingStrategy+") + parent->toString(); }

    private:
        BeadingStrategy* parent;
        coord_t optimal_width_outer;
        coord_t optimal_width_inner;
    };

} // namespace cura
#endif // INWARD_DISTRIBUTED_BEADING_STRATEGY_H
