//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BEADING_STRATEGY_FACTORY_H
#define BEADING_STRATEGY_FACTORY_H

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
extern int max_bead_count;
extern coord_t default_transition_length;

enum class StrategyType
{
    Naive,
    NaiveStrategy,
    Center,
    Distributed,
    InwardDistributed,
    LimitedDistributed,
    COUNT
};

StrategyType toStrategyType(char c);

std::string to_string(StrategyType type);

class BeadingStrategyFactory
{
public:
    static BeadingStrategy* makeStrategy(StrategyType type, coord_t prefered_bead_width = MM2INT(0.5), float transitioning_angle = M_PI / 4, std::optional<coord_t> min_bead_width = NULL, std::optional<coord_t> min_feature_size = NULL);
};

} // namespace cura
#endif // BEADING_STRATEGY_FACTORY_H
