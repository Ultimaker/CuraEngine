// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BEADING_STRATEGY_FACTORY_H
#define BEADING_STRATEGY_FACTORY_H

#include "../settings/types/Ratio.h"
#include "BeadingStrategy.h"

namespace cura
{

enum class StrategyType
{
    Center,
    Distributed,
    InwardDistributed,
    None,
    COUNT
};

class BeadingStrategyFactory
{
public:
    static BeadingStrategyPtr makeStrategy
    (
        const StrategyType type,
        const coord_t preferred_bead_width_outer = MM2INT(0.5),
        const coord_t preferred_bead_width_inner = MM2INT(0.5),
        const coord_t preferred_transition_length = MM2INT(0.4),
        const float transitioning_angle = M_PI_4,
        const bool print_thin_walls = false,
        const coord_t min_bead_width = 0,
        const coord_t min_feature_size = 0,
        const Ratio wall_split_middle_threshold = 0.5_r,
        const Ratio wall_add_middle_threshold = 0.5_r,
        const coord_t max_bead_count = 0,
        const coord_t outer_wall_offset = 0,
        const int inward_distributed_center_wall_count = 2,
        const double minimum_variable_line_width = 0.5
    );
};

} // namespace cura
#endif // BEADING_STRATEGY_FACTORY_H
