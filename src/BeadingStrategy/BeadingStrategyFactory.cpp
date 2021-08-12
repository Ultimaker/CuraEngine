//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategyFactory.h"

#include "LimitedBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "WideningBeadingStrategy.h"
#include "DistributedBeadingStrategy.h"
#include "RedistributeBeadingStrategy.h"
#include "OuterWallInsetBeadingStrategy.h"

#include <limits>

namespace cura
{

coord_t getWeightedAverage(const coord_t preferred_bead_width_outer, const coord_t preferred_bead_width_inner, const coord_t max_bead_count)
{
    if (max_bead_count > 2)
    {
        return ((preferred_bead_width_outer * 2) + preferred_bead_width_inner * (max_bead_count - 2)) / max_bead_count;
    }
    if (max_bead_count <= 0)
    {
        return preferred_bead_width_inner;
    }
    return preferred_bead_width_outer;
}

BeadingStrategy* BeadingStrategyFactory::makeStrategy
(
    const StrategyType type,
    const coord_t preferred_bead_width_outer,
    const coord_t preferred_bead_width_inner,
    const coord_t preferred_transition_length,
    const float transitioning_angle,
    const bool print_thin_walls,
    const coord_t min_bead_width,
    const coord_t min_feature_size,
    const Ratio wall_split_middle_threshold,
    const Ratio wall_add_middle_threshold,
    const coord_t max_bead_count,
    const coord_t outer_wall_offset,
    const int inward_distributed_center_wall_count,
    const double minimum_variable_line_width
)
{
    const coord_t bar_preferred_wall_width = getWeightedAverage(preferred_bead_width_outer, preferred_bead_width_inner, max_bead_count);
    BeadingStrategy* ret = nullptr;
    switch (type)
    {
        case StrategyType::Center:
            ret = new CenterDeviationBeadingStrategy(bar_preferred_wall_width, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold);
            break;
        case StrategyType::Distributed:
            ret = new DistributedBeadingStrategy(bar_preferred_wall_width, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, std::numeric_limits<int>::max());
            break;
        case StrategyType::InwardDistributed:
            ret = new DistributedBeadingStrategy(bar_preferred_wall_width, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, inward_distributed_center_wall_count);
            break;
        default:
            logError("Cannot make strategy!\n");
            return nullptr;
    }
    
    if(print_thin_walls)
    {
        logDebug("Applying the Widening Beading meta-strategy with minimum input width %d and minimum output width %d.", min_feature_size, min_bead_width);
        ret = new WideningBeadingStrategy(ret, min_feature_size, min_bead_width);
    }
    if (max_bead_count > 0)
    {
        logDebug("Applying the Redistribute meta-strategy with outer-wall width = %d, inner-wall width = %d", preferred_bead_width_outer, preferred_bead_width_inner);
        ret = new RedistributeBeadingStrategy(preferred_bead_width_outer, preferred_bead_width_inner, minimum_variable_line_width, ret);
        //Apply the LimitedBeadingStrategy last, since that adds a 0-width marker wall which other beading strategies shouldn't touch.
        logDebug("Applying the Limited Beading meta-strategy with maximum bead count = %d.", max_bead_count);
        ret = new LimitedBeadingStrategy(max_bead_count, ret);
    }
    
    if (outer_wall_offset > 0)
    {
        logDebug("Applying the OuterWallOffset meta-strategy with offset = %d.", outer_wall_offset);
        ret = new OuterWallInsetBeadingStrategy(outer_wall_offset, ret);
    }
    return ret;
}
} // namespace cura
