// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INNER_WALL_INSET_BEADING_STRATEGY_H
#define INNER_WALL_INSET_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace cura
{
/*
 * This is a meta strategy that allows for the inner wall to be inset towards the inside of the model.
 */
class InnerWallInsetBeadingStrategy : public BeadingStrategy
{
public:
    InnerWallInsetBeadingStrategy(coord_t inner_wall_offset, BeadingStrategyPtr parent);

    virtual ~InnerWallInsetBeadingStrategy() = default;

    Beading compute(coord_t thickness, coord_t bead_count) const override;

    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
    coord_t getTransitioningLength(coord_t lower_bead_count) const override;

    std::string toString() const override;

private:
    BeadingStrategyPtr parent_;
    coord_t inner_wall_offset_;
};
} // namespace cura
#endif // INNER_WALL_INSET_BEADING_STRATEGY_H
