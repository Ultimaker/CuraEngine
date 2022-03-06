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
    if(max_bead_count > preferred_bead_width_outer - preferred_bead_width_inner)
    {
        //The difference between outer and inner bead width would be spread out across so many lines that rounding errors would destroy the difference.
        //Also catches the case of max_bead_count being "infinite" (max integer).
        return (preferred_bead_width_outer + preferred_bead_width_inner) / 2;
    }
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

BeadingStrategyPtr BeadingStrategyFactory::makeStrategy
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
    using std::make_unique;
    using std::move;
    const coord_t bar_preferred_wall_width = getWeightedAverage(preferred_bead_width_outer, preferred_bead_width_inner, max_bead_count);
    BeadingStrategyPtr ret;
    switch (type)
    {
        case StrategyType::Center:
            ret = make_unique<CenterDeviationBeadingStrategy>(bar_preferred_wall_width, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold);
            break;
        case StrategyType::Distributed:
            ret = make_unique<DistributedBeadingStrategy>(bar_preferred_wall_width, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, std::numeric_limits<int>::max());
            break;
        case StrategyType::InwardDistributed:
            ret = make_unique<DistributedBeadingStrategy>(bar_preferred_wall_width, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, inward_distributed_center_wall_count);
            break;
        default:
            logError("Cannot make strategy!\n");
            return nullptr;
    }
    
    if(print_thin_walls)
    {
        logDebug("Applying the Widening Beading meta-strategy with minimum input width %lld and minimum output width %lld.\n", min_feature_size, min_bead_width);
        ret = make_unique<WideningBeadingStrategy>(move(ret), min_feature_size, min_bead_width);
    }
    if (max_bead_count > 0)
    {
        logDebug("Applying the Redistribute meta-strategy with outer-wall width = %lld, inner-wall width = %lld\n", preferred_bead_width_outer, preferred_bead_width_inner);
        ret = make_unique<RedistributeBeadingStrategy>(preferred_bead_width_outer, preferred_bead_width_inner, minimum_variable_line_width, move(ret));
        //Apply the LimitedBeadingStrategy last, since that adds a 0-width marker wall which other beading strategies shouldn't touch.
        logDebug("Applying the Limited Beading meta-strategy with maximum bead count = %lld.\n", max_bead_count);
        ret = make_unique<LimitedBeadingStrategy>(max_bead_count, move(ret));
    }
    
    if (outer_wall_offset > 0)
    {
        logDebug("Applying the OuterWallOffset meta-strategy with offset = %lld.\n", outer_wall_offset);
        ret = make_unique<OuterWallInsetBeadingStrategy>(outer_wall_offset, move(ret));
    }
    return ret;
}
} // namespace cura
