// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "beading_strategy/BeadingStrategy.h"

namespace curaengine::beading_strategy
{

/*!
 * This is a meta-strategy that can be applied on any other beading strategy. If
 * the part is thinner than a single line, this strategy adjusts the part so
 * that it becomes the minimum thickness of one line.
 *
 * This way, tiny pieces that are smaller than a single line will still be
 * printed.
 */
class WideningBeadingStrategy : public BeadingStrategy
{
public:
    /*!
     * Takes responsibility for deleting \param parent
     */
    WideningBeadingStrategy(BeadingStrategyPtr parent, const types::coord_t min_input_width, const types::coord_t min_output_width);

    virtual ~WideningBeadingStrategy() override = default;

    Beading compute(types::coord_t thickness, types::coord_t bead_count) const override;
    types::coord_t getOptimalThickness(types::coord_t bead_count) const override;
    types::coord_t getTransitionThickness(types::coord_t lower_bead_count) const override;
    types::coord_t getOptimalBeadCount(types::coord_t thickness) const override;
    types::coord_t getTransitioningLength(types::coord_t lower_bead_count) const override;
    double getTransitionAnchorPos(types::coord_t lower_bead_count) const override;
    std::vector<types::coord_t> getNonlinearThicknesses(types::coord_t lower_bead_count) const override;
    std::string toString() const override;

protected:
    BeadingStrategyPtr parent_;
    const types::coord_t min_input_width_;
    const types::coord_t min_output_width_;
};

} // namespace curaengine::beading_strategy
