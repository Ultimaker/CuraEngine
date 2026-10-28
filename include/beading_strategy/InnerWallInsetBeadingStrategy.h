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
    InnerWallInsetBeadingStrategy(utils::coord_t inner_wall_offset, BeadingStrategyPtr parent);

    virtual ~InnerWallInsetBeadingStrategy() = default;

    Beading compute(utils::coord_t thickness, utils::coord_t bead_count) const override;

    utils::coord_t getOptimalThickness(utils::coord_t bead_count) const override;
    utils::coord_t getTransitionThickness(utils::coord_t lower_bead_count) const override;
    utils::coord_t getOptimalBeadCount(utils::coord_t thickness) const override;
    utils::coord_t getTransitioningLength(utils::coord_t lower_bead_count) const override;

    std::string toString() const override;

private:
    BeadingStrategyPtr parent_;
    utils::coord_t inner_wall_offset_;
};
} // namespace curaengine::beading_strategy
