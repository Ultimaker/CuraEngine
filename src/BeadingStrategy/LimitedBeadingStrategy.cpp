// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "BeadingStrategy/LimitedBeadingStrategy.h"

#include <cassert>

#include <spdlog/spdlog.h>

namespace cura
{

std::string LimitedBeadingStrategy::toString() const
{
    return std::string("LimitedBeadingStrategy+") + parent_->toString();
}

coord_t LimitedBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return parent_->getTransitioningLength(lower_bead_count);
}

double LimitedBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    return parent_->getTransitionAnchorPos(lower_bead_count);
}

LimitedBeadingStrategy::LimitedBeadingStrategy(const coord_t max_bead_count, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , max_bead_count_(max_bead_count)
    , parent_(std::move(parent))
{
    if (max_bead_count % 2 == 1)
    {
        RUN_ONCE(spdlog::warn("LimitedBeadingStrategy with odd bead count is odd indeed!"));
    }
}

LimitedBeadingStrategy::Beading LimitedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (bead_count <= max_bead_count_)
    {
        Beading ret = parent_->compute(thickness, bead_count);
        bead_count = ret.toolpath_locations.size();

        if (bead_count % 2 == 0 && bead_count == max_bead_count_)
        {
            const coord_t innermost_toolpath_location = ret.toolpath_locations[max_bead_count_ / 2 - 1];
            const coord_t innermost_toolpath_width = ret.bead_widths[max_bead_count_ / 2 - 1];
            ret.toolpath_locations.insert(ret.toolpath_locations.begin() + max_bead_count_ / 2, innermost_toolpath_location + innermost_toolpath_width / 2);
            ret.bead_widths.insert(ret.bead_widths.begin() + max_bead_count_ / 2, 0);
        }
        return ret;
    }
    assert(bead_count == max_bead_count_ + 1);
    if (bead_count != max_bead_count_ + 1)
    {
        RUN_ONCE(spdlog::warn("Too many beads! {} != {}", bead_count, max_bead_count_ + 1));
    }

    coord_t optimal_thickness = parent_->getOptimalThickness(max_bead_count_);
    Beading ret = parent_->compute(optimal_thickness, max_bead_count_);
    bead_count = ret.toolpath_locations.size();
    ret.left_over += thickness - ret.total_thickness;
    ret.total_thickness = thickness;

    // Enforce symmetry
    if (bead_count % 2 == 1)
    {
        ret.toolpath_locations[bead_count / 2] = thickness / 2;
        ret.bead_widths[bead_count / 2] = thickness - optimal_thickness;
    }
    for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
    {
        ret.toolpath_locations[bead_count - 1 - bead_idx] = thickness - ret.toolpath_locations[bead_idx];
    }

    // Create a "fake" inner wall with 0 width to indicate the edge of the walled area.
    // This wall can then be used by other structures to e.g. fill the infill area adjacent to the variable-width walls.
    coord_t innermost_toolpath_location = ret.toolpath_locations[max_bead_count_ / 2 - 1];
    coord_t innermost_toolpath_width = ret.bead_widths[max_bead_count_ / 2 - 1];
    ret.toolpath_locations.insert(ret.toolpath_locations.begin() + max_bead_count_ / 2, innermost_toolpath_location + innermost_toolpath_width / 2);
    ret.bead_widths.insert(ret.bead_widths.begin() + max_bead_count_ / 2, 0);

    // Symmetry on both sides. Symmetry is guaranteed since this code is stopped early if the bead_count <= max_bead_count, and never reaches this point then.
    const size_t opposite_bead = bead_count - (max_bead_count_ / 2 - 1);
    innermost_toolpath_location = ret.toolpath_locations[opposite_bead];
    innermost_toolpath_width = ret.bead_widths[opposite_bead];
    ret.toolpath_locations.insert(ret.toolpath_locations.begin() + opposite_bead, innermost_toolpath_location - innermost_toolpath_width / 2);
    ret.bead_widths.insert(ret.bead_widths.begin() + opposite_bead, 0);

    return ret;
}

coord_t LimitedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    if (bead_count <= max_bead_count_)
    {
        return parent_->getOptimalThickness(bead_count);
    }
    return 10000000; // 10 meter
}

coord_t LimitedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    if (lower_bead_count < max_bead_count_)
    {
        return parent_->getTransitionThickness(lower_bead_count);
    }
    if (lower_bead_count == max_bead_count_)
    {
        return parent_->getOptimalThickness(lower_bead_count + 1) - 10;
    }
    return 9000000; // 9 meter
}

coord_t LimitedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    coord_t parent_bead_count = parent_->getOptimalBeadCount(thickness);
    if (parent_bead_count <= max_bead_count_)
    {
        return parent_->getOptimalBeadCount(thickness);
    }
    else if (parent_bead_count == max_bead_count_ + 1)
    {
        if (thickness < parent_->getOptimalThickness(max_bead_count_ + 1) - 10)
            return max_bead_count_;
        else
            return max_bead_count_ + 1;
    }
    else
        return max_bead_count_ + 1;
}

} // namespace cura
