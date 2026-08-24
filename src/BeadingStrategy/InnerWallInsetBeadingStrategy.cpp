// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategy/InnerWallInsetBeadingStrategy.h"

#include <algorithm>

#include <range/v3/view/drop.hpp>

namespace cura
{
InnerWallInsetBeadingStrategy::InnerWallInsetBeadingStrategy(coord_t inner_wall_offset, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , parent_(std::move(parent))
    , inner_wall_offset_(inner_wall_offset)
{
    name_ = "InnerWallInsetBeadingStrategy";
}


coord_t InnerWallInsetBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return parent_->getOptimalThickness(bead_count);
}

coord_t InnerWallInsetBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    return parent_->getTransitionThickness(lower_bead_count);
}

coord_t InnerWallInsetBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    return parent_->getOptimalBeadCount(thickness);
}

coord_t InnerWallInsetBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return parent_->getTransitioningLength(lower_bead_count);
}

std::string InnerWallInsetBeadingStrategy::toString() const
{
    return std::string("InnerWallInsetBeadingStrategy+") + parent_->toString();
}

BeadingStrategy::Beading InnerWallInsetBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret = parent_->compute(thickness, bead_count);

    // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signaling' walls.
    bead_count = std::count_if(
        ret.bead_widths.begin(),
        ret.bead_widths.end(),
        [](const coord_t width)
        {
            return width > 0;
        });

    // No need to apply any inset if there is just a single wall.
    if (bead_count < 2)
    {
        return ret;
    }

    for (auto& location : ret.toolpath_locations | ranges::views::drop(1))
    {
        location += inner_wall_offset_;
    }

    return ret;
}

} // namespace cura
