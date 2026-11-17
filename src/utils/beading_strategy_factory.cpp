// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/beading_strategy_factory.h"

#include <limits>

#include <spdlog/spdlog.h>

#include "beading_strategy/DistributedBeadingStrategy.h"
#include "beading_strategy/InnerWallInsetBeadingStrategy.h"
#include "beading_strategy/LimitedBeadingStrategy.h"
#include "beading_strategy/OuterWallInsetBeadingStrategy.h"
#include "beading_strategy/RedistributeBeadingStrategy.h"
#include "beading_strategy/WideningBeadingStrategy.h"

namespace curaengine::utils::beading_strategy_factory
{
using namespace beading_strategy;
using namespace types;
using namespace utils;

BeadingStrategyPtr makeStrategy(
    const coord_t preferred_bead_width_outer,
    const coord_t preferred_bead_width_inner,
    const coord_t preferred_transition_length,
    const double transitioning_angle,
    const bool print_thin_walls,
    const coord_t min_bead_width,
    const coord_t min_feature_size,
    const Ratio wall_split_middle_threshold,
    const Ratio wall_add_middle_threshold,
    const coord_t max_bead_count,
    const coord_t outer_wall_offset,
    const coord_t inner_wall_offset,
    const int inward_distributed_center_wall_count,
    const Ratio minimum_variable_line_ratio)
{
    BeadingStrategyPtr ret = std::make_unique<DistributedBeadingStrategy>(
        preferred_bead_width_inner,
        preferred_transition_length,
        transitioning_angle,
        wall_split_middle_threshold,
        wall_add_middle_threshold,
        inward_distributed_center_wall_count);
    spdlog::debug("Applying the Redistribute meta-strategy with outer-wall width = {}, inner-wall width = {}", preferred_bead_width_outer, preferred_bead_width_inner);
    ret = std::make_unique<RedistributeBeadingStrategy>(preferred_bead_width_outer, minimum_variable_line_ratio, std::move(ret));

    if (print_thin_walls)
    {
        spdlog::debug("Applying the Widening Beading meta-strategy with minimum input width {} and minimum output width {}.", min_feature_size, min_bead_width);
        ret = std::make_unique<WideningBeadingStrategy>(std::move(ret), min_feature_size, min_bead_width);
    }
    if (outer_wall_offset > 0)
    {
        spdlog::debug("Applying the OuterWallOffset meta-strategy with offset = {}", outer_wall_offset);
        ret = std::make_unique<OuterWallInsetBeadingStrategy>(outer_wall_offset, std::move(ret));
    }
    if (inner_wall_offset != 0)
    {
        spdlog::debug("Applying the InnerWallOffset meta-strategy with offset = {}", inner_wall_offset);
        ret = std::make_unique<InnerWallInsetBeadingStrategy>(inner_wall_offset, std::move(ret));
    }

    // Apply the LimitedBeadingStrategy last, since that adds a 0-width marker wall which other beading strategies shouldn't touch.
    spdlog::debug("Applying the Limited Beading meta-strategy with maximum bead count = {}", max_bead_count);
    ret = std::make_unique<LimitedBeadingStrategy>(max_bead_count, std::move(ret));
    return ret;
}
} // namespace curaengine::utils::beading_strategy_factory
