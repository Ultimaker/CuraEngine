//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BEADING_STRATEGY_FACTORY_H
#define BEADING_STRATEGY_FACTORY_H

#include <memory>

#include "../utils/optional.h"  // until the move to C++17

#include "BeadingStrategy.h"

namespace cura
{

extern double inward_distributed_center_size;

enum class StrategyType
{
    Center,
    Distributed,
    InwardDistributed,
    None,
    COUNT
};

StrategyType toStrategyType(char c);

std::string to_string(StrategyType type);

class BeadingStrategyFactory
{
public:
    static BeadingStrategy* makeStrategy(const StrategyType type,
        const coord_t preferred_bead_width_outer = MM2INT(0.5),
        const coord_t preferred_bead_width_inner = MM2INT(0.5),
        const coord_t preferred_transition_length = 400,
        const float transitioning_angle = M_PI_4,
        const bool print_thin_walls = false,
        const coord_t min_bead_width = 0,
        const coord_t min_feature_size = 0,
        const coord_t max_bead_count = 0);
};

} // namespace cura
#endif // BEADING_STRATEGY_FACTORY_H
