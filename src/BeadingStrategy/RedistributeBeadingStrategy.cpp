// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategy/RedistributeBeadingStrategy.h"

#include <algorithm>
#include <numeric>

namespace cura
{

RedistributeBeadingStrategy::RedistributeBeadingStrategy(const coord_t optimal_width_outer, const Ratio minimum_variable_line_ratio, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , parent_(std::move(parent))
    , optimal_width_outer_(optimal_width_outer)
    , minimum_variable_line_ratio_(minimum_variable_line_ratio)
{
    name_ = "RedistributeBeadingStrategy";
}

coord_t RedistributeBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    const coord_t inner_bead_count = std::max(static_cast<coord_t>(0), bead_count - 2);
    const coord_t outer_bead_count = bead_count - inner_bead_count;
    return parent_->getOptimalThickness(inner_bead_count) + optimal_width_outer_ * outer_bead_count;
}

coord_t RedistributeBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
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

coord_t RedistributeBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
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

coord_t RedistributeBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return parent_->getTransitioningLength(lower_bead_count);
}

double RedistributeBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    return parent_->getTransitionAnchorPos(lower_bead_count);
}

std::string RedistributeBeadingStrategy::toString() const
{
    return toString() + parent_->toString();
}

BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
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
    const coord_t inner_bead_count = bead_count - 2;
    const coord_t inner_thickness = thickness - 2 * optimal_width_outer_;
    if (inner_bead_count > 0 && inner_thickness > 0)
    {
        ret = parent_->compute(inner_thickness, inner_bead_count);
        for (auto& toolpath_location : ret.toolpath_locations)
        {
            toolpath_location += optimal_width_outer_;
        }
    }

    // Insert the outer wall(s) around the previously computed inner wall(s), which may be empty:
    const coord_t actual_outer_thickness = bead_count > 2 ? std::min(thickness / 2, optimal_width_outer_) : thickness / bead_count;
    ret.bead_widths.insert(ret.bead_widths.begin(), actual_outer_thickness);
    ret.toolpath_locations.insert(ret.toolpath_locations.begin(), actual_outer_thickness / 2);
    if (bead_count > 1)
    {
        ret.bead_widths.push_back(actual_outer_thickness);
        ret.toolpath_locations.push_back(thickness - actual_outer_thickness / 2);
    }

    // Ensure correct total and left over thickness.
    ret.total_thickness = thickness;
    ret.left_over = thickness - std::accumulate(ret.bead_widths.cbegin(), ret.bead_widths.cend(), static_cast<coord_t>(0));
    return ret;
}

} // namespace cura
