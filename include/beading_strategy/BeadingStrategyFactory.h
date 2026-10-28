// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <numbers>

#include "beading_strategy/BeadingStrategy.h"
#include "types/Ratio.h"

namespace curaengine::beading_strategy
{
using types::operator""_r;


class BeadingStrategyFactory
{
public:
    static BeadingStrategyPtr makeStrategy(
        const utils::coord_t preferred_bead_width_outer = MM2INT(0.5),
        const utils::coord_t preferred_bead_width_inner = MM2INT(0.5),
        const utils::coord_t preferred_transition_length = MM2INT(0.4),
        const double transitioning_angle = std::numbers::pi / 4.0,
        const bool print_thin_walls = false,
        const utils::coord_t min_bead_width = 0,
        const utils::coord_t min_feature_size = 0,
        const types::Ratio wall_split_middle_threshold = 0.5_r,
        const types::Ratio wall_add_middle_threshold = 0.5_r,
        const utils::coord_t max_bead_count = 0,
        const utils::coord_t outer_wall_offset = 0,
        const utils::coord_t inner_wall_offset = 0,
        const int inward_distributed_center_wall_count = 2,
        const types::Ratio minimum_variable_line_ratio = 0.5);
};

} // namespace curaengine::beading_strategy
