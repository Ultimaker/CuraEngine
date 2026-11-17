// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "beading_strategy/BeadingStrategy.h"

namespace curaengine::beading_strategy
{
/*
 * This is a meta strategy that allows for the inner wall to be inset towards the inside of the model.
 */
class InnerWallInsetBeadingStrategy : public BeadingStrategy
{
public:
    InnerWallInsetBeadingStrategy(types::coord_t inner_wall_offset, BeadingStrategyPtr parent);

    virtual ~InnerWallInsetBeadingStrategy() = default;

    Beading compute(types::coord_t thickness, types::coord_t bead_count) const override;

    types::coord_t getOptimalThickness(types::coord_t bead_count) const override;
    types::coord_t getTransitionThickness(types::coord_t lower_bead_count) const override;
    types::coord_t getOptimalBeadCount(types::coord_t thickness) const override;
    types::coord_t getTransitioningLength(types::coord_t lower_bead_count) const override;

    std::string toString() const override;

private:
    BeadingStrategyPtr parent_;
    types::coord_t inner_wall_offset_;
};
} // namespace curaengine::beading_strategy
