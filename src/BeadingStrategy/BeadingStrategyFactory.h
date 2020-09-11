//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BEADING_STRATEGY_FACTORY_H
#define BEADING_STRATEGY_FACTORY_H

#include <memory>

#include "../utils/optional.h"  // until the move to C++17

#include "BeadingStrategy.h"
#include "InwardDistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "LimitedBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "WideningBeadingStrategy.h"
#include "NaiveBeadingStrategy.h"
#include "DistributedBeadingStrategy.h"

namespace cura
{

extern double inward_distributed_center_size;

enum class StrategyType
{
    Naive,
    Center,
    Distributed,
    InwardDistributed,
    LimitedDistributed,
    None,
    COUNT
};

StrategyType toStrategyType(char c);

std::string to_string(StrategyType type);

class BeadingStrategyFactory
{
public:
    static BeadingStrategy* makeStrategy(StrategyType type, coord_t preferred_bead_width = MM2INT(0.5), coord_t preferred_transition_length = 400, float transitioning_angle = M_PI / 4, const std::unique_ptr<coord_t>& min_bead_width = nullptr, const std::unique_ptr<coord_t>& min_feature_size = nullptr,  const coord_t max_bead_count = 0);
};

} // namespace cura
#endif // BEADING_STRATEGY_FACTORY_H
