//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategyFactory.h"

#include "LimitedBeadingStrategy.h"
#include "WideningBeadingStrategy.h"
#include "DistributedBeadingStrategy.h"
#include "RedistributeBeadingStrategy.h"
#include "OuterWallInsetBeadingStrategy.h"

#include <limits>

namespace cura
{

BeadingStrategyPtr BeadingStrategyFactory::makeStrategy
(
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
    const Ratio minimum_variable_line_ratio
)
{
    using std::make_unique;
    using std::move;
    BeadingStrategyPtr ret = make_unique<DistributedBeadingStrategy>(preferred_bead_width_inner, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, inward_distributed_center_wall_count);
    logDebug("Applying the Redistribute meta-strategy with outer-wall width = %d, inner-wall width = %d\n", preferred_bead_width_outer, preferred_bead_width_inner);
    ret = make_unique<RedistributeBeadingStrategy>(preferred_bead_width_outer, minimum_variable_line_ratio, move(ret));

    if(print_thin_walls)
    {
        logDebug("Applying the Widening Beading meta-strategy with minimum input width %d and minimum output width %d.\n", min_feature_size, min_bead_width);
        ret = make_unique<WideningBeadingStrategy>(move(ret), min_feature_size, min_bead_width);
    }
    if (outer_wall_offset > 0)
    {
        logDebug("Applying the OuterWallOffset meta-strategy with offset = %d.\n", outer_wall_offset);
        ret = make_unique<OuterWallInsetBeadingStrategy>(outer_wall_offset, move(ret));
    }

    //Apply the LimitedBeadingStrategy last, since that adds a 0-width marker wall which other beading strategies shouldn't touch.
    logDebug("Applying the Limited Beading meta-strategy with maximum bead count = %d.\n", max_bead_count);
    ret = make_unique<LimitedBeadingStrategy>(max_bead_count, move(ret));
    return ret;
}
} // namespace cura
