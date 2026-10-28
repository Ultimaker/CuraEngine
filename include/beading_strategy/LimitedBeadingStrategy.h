// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "beading_strategy/BeadingStrategy.h"

namespace curaengine::beading_strategy
{

/*!
 * This is a meta-strategy that can be applied on top of any other beading
 * strategy, which limits the thickness of the walls to the thickness that the
 * lines can reasonably print.
 *
 * The width of the wall is limited to the maximum number of contours times the
 * maximum width of each of these contours.
 *
 * If the width of the wall gets limited, this strategy outputs one additional
 * bead with 0 width. This bead is used to denote the limits of the walled area.
 * Other structures can then use this border to align their structures to, such
 * as to create correctly overlapping infill or skin, or to align the infill
 * pattern to any extra infill walls.
 */
class LimitedBeadingStrategy : public BeadingStrategy
{
public:
    LimitedBeadingStrategy(const utils::coord_t max_bead_count, BeadingStrategyPtr parent);

    virtual ~LimitedBeadingStrategy() override = default;

    Beading compute(utils::coord_t thickness, utils::coord_t bead_count) const override;
    utils::coord_t getOptimalThickness(utils::coord_t bead_count) const override;
    utils::coord_t getTransitionThickness(utils::coord_t lower_bead_count) const override;
    utils::coord_t getOptimalBeadCount(utils::coord_t thickness) const override;
    std::string toString() const override;

    utils::coord_t getTransitioningLength(utils::coord_t lower_bead_count) const override;

    double getTransitionAnchorPos(utils::coord_t lower_bead_count) const override;

protected:
    const utils::coord_t max_bead_count_;
    const BeadingStrategyPtr parent_;
};


} // namespace curaengine::beading_strategy
