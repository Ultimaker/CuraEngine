// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "beading_strategy/RedistributeBeadingStrategy.h"

#include <algorithm>
#include <numeric>

namespace curaengine::beading_strategy
{

RedistributeBeadingStrategy::RedistributeBeadingStrategy(const ::curaengine::utils::coord_t optimal_width_outer, const ::curaengine::types::Ratio minimum_variable_line_ratio, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , parent_(std::move(parent))
    , optimal_width_outer_(optimal_width_outer)
    , minimum_variable_line_ratio_(minimum_variable_line_ratio)
{
    name_ = "RedistributeBeadingStrategy";
}

::curaengine::utils::coord_t RedistributeBeadingStrategy::getOptimalThickness(::curaengine::utils::coord_t bead_count) const
{
    const ::curaengine::utils::coord_t inner_bead_count = std::max(static_cast<::curaengine::utils::coord_t>(0), bead_count - 2);
    const ::curaengine::utils::coord_t outer_bead_count = bead_count - inner_bead_count;
    return parent_->getOptimalThickness(inner_bead_count) + optimal_width_outer_ * outer_bead_count;
}

::curaengine::utils::coord_t RedistributeBeadingStrategy::getTransitionThickness(::curaengine::utils::coord_t lower_bead_count) const
{
    switch (lower_bead_count)
    {
    case 0:
        return minimum_variable_line_ratio_ * optimal_width_outer_;
    case 1:
        return (1.0 + parent_->getSplitMiddleThreshold()) * optimal_width_outer_;
    default:
        return parent_->getTransitionThickness(lower_bead_count - 2) + 2 * optimal_width_outer_;
    }
}

::curaengine::utils::coord_t RedistributeBeadingStrategy::getOptimalBeadCount(::curaengine::utils::coord_t thickness) const
{
    if (thickness < minimum_variable_line_ratio_ * optimal_width_outer_)
    {
        return 0;
    }
    if (thickness <= 2 * optimal_width_outer_)
    {
        return thickness > (1.0 + parent_->getSplitMiddleThreshold()) * optimal_width_outer_ ? 2 : 1;
    }
    return parent_->getOptimalBeadCount(thickness - 2 * optimal_width_outer_) + 2;
}

::curaengine::utils::coord_t RedistributeBeadingStrategy::getTransitioningLength(::curaengine::utils::coord_t lower_bead_count) const
{
    return parent_->getTransitioningLength(lower_bead_count);
}

double RedistributeBeadingStrategy::getTransitionAnchorPos(::curaengine::utils::coord_t lower_bead_count) const
{
    return parent_->getTransitionAnchorPos(lower_bead_count);
}

std::string RedistributeBeadingStrategy::toString() const
{
    return toString() + parent_->toString();
}

BeadingStrategy::Beading RedistributeBeadingStrategy::compute(::curaengine::utils::coord_t thickness, ::curaengine::utils::coord_t bead_count) const
{
    Beading ret;

    // Take care of all situations in which no lines are actually produced:
    if (bead_count == 0 || thickness < minimum_variable_line_ratio_ * optimal_width_outer_)
    {
        ret.left_over = thickness;
        ret.total_thickness = thickness;
        return ret;
    }

    // Compute the beadings of the inner walls, if any:
    const ::curaengine::utils::coord_t inner_bead_count = bead_count - 2;
    const ::curaengine::utils::coord_t inner_thickness = thickness - 2 * optimal_width_outer_;
    if (inner_bead_count > 0 && inner_thickness > 0)
    {
        ret = parent_->compute(inner_thickness, inner_bead_count);
        for (auto& toolpath_location : ret.toolpath_locations)
        {
            toolpath_location += optimal_width_outer_;
        }
    }

    // Insert the outer wall(s) around the previously computed inner wall(s), which may be empty:
    const ::curaengine::utils::coord_t actual_outer_thickness = bead_count > 2 ? std::min(thickness / 2, optimal_width_outer_) : thickness / bead_count;
    ret.bead_widths.insert(ret.bead_widths.begin(), actual_outer_thickness);
    ret.toolpath_locations.insert(ret.toolpath_locations.begin(), actual_outer_thickness / 2);
    if (bead_count > 1)
    {
        ret.bead_widths.push_back(actual_outer_thickness);
        ret.toolpath_locations.push_back(thickness - actual_outer_thickness / 2);
    }

    // Ensure correct total and left over thickness.
    ret.total_thickness = thickness;
    ret.left_over = thickness - std::accumulate(ret.bead_widths.cbegin(), ret.bead_widths.cend(), static_cast<::curaengine::utils::coord_t>(0));
    return ret;
}

} // namespace curaengine::beading_strategy
