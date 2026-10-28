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
    WideningBeadingStrategy(BeadingStrategyPtr parent, const ::curaengine::utils::coord_t min_input_width, const ::curaengine::utils::coord_t min_output_width);

    virtual ~WideningBeadingStrategy() override = default;

    Beading compute(::curaengine::utils::coord_t thickness, ::curaengine::utils::coord_t bead_count) const override;
    ::curaengine::utils::coord_t getOptimalThickness(::curaengine::utils::coord_t bead_count) const override;
    ::curaengine::utils::coord_t getTransitionThickness(::curaengine::utils::coord_t lower_bead_count) const override;
    ::curaengine::utils::coord_t getOptimalBeadCount(::curaengine::utils::coord_t thickness) const override;
    ::curaengine::utils::coord_t getTransitioningLength(::curaengine::utils::coord_t lower_bead_count) const override;
    double getTransitionAnchorPos(::curaengine::utils::coord_t lower_bead_count) const override;
    std::vector<::curaengine::utils::coord_t> getNonlinearThicknesses(::curaengine::utils::coord_t lower_bead_count) const override;
    std::string toString() const override;

protected:
    BeadingStrategyPtr parent_;
    const ::curaengine::utils::coord_t min_input_width_;
    const ::curaengine::utils::coord_t min_output_width_;
};

} // namespace curaengine::beading_strategy
