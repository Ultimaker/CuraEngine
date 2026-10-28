// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "beading_strategy/OuterWallInsetBeadingStrategy.h"

#include <algorithm>

namespace curaengine::beading_strategy
{
OuterWallInsetBeadingStrategy::OuterWallInsetBeadingStrategy(::curaengine::utils::coord_t outer_wall_offset, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , parent_(std::move(parent))
    , outer_wall_offset_(outer_wall_offset)
{
    name_ = "OuterWallOfsetBeadingStrategy";
}


::curaengine::utils::coord_t OuterWallInsetBeadingStrategy::getOptimalThickness(::curaengine::utils::coord_t bead_count) const
{
    return parent_->getOptimalThickness(bead_count);
}

::curaengine::utils::coord_t OuterWallInsetBeadingStrategy::getTransitionThickness(::curaengine::utils::coord_t lower_bead_count) const
{
    return parent_->getTransitionThickness(lower_bead_count);
}

::curaengine::utils::coord_t OuterWallInsetBeadingStrategy::getOptimalBeadCount(::curaengine::utils::coord_t thickness) const
{
    return parent_->getOptimalBeadCount(thickness);
}

::curaengine::utils::coord_t OuterWallInsetBeadingStrategy::getTransitioningLength(::curaengine::utils::coord_t lower_bead_count) const
{
    return parent_->getTransitioningLength(lower_bead_count);
}

std::string OuterWallInsetBeadingStrategy::toString() const
{
    return std::string("OuterWallOfsetBeadingStrategy+") + parent_->toString();
}

BeadingStrategy::Beading OuterWallInsetBeadingStrategy::compute(::curaengine::utils::coord_t thickness, ::curaengine::utils::coord_t bead_count) const
{
    Beading ret = parent_->compute(thickness, bead_count);

    // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signaling' walls.
    bead_count = std::count_if(
        ret.bead_widths.begin(),
        ret.bead_widths.end(),
        [](const ::curaengine::utils::coord_t width)
        {
            return width > 0;
        });

    // No need to apply any inset if there is just a single wall.
    if (bead_count < 2)
    {
        return ret;
    }

    // Actually move the outer wall inside. Ensure that the outer wall never goes beyond the middle line.
    ret.toolpath_locations[0] = std::min(ret.toolpath_locations[0] + outer_wall_offset_, thickness / 2);
    return ret;
}

} // namespace curaengine::beading_strategy
