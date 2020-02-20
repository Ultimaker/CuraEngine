//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_STRATEGY_HELPER_H
#define BEADING_STRATEGY_HELPER_H

#include <optional>

#include "BeadingStrategy.h"
#include "InwardDistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "LimitedBeadingStrategy.h"
#include "SingleBeadBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "OutlineAccuracyBeadingStrategy.h"
#include "WideningBeadingStrategy.h"
#include "ConstantBeadingStrategy.h"
#include "NaiveBeadingStrategy.h"
#include "DistributedBeadingStrategy.h"

namespace arachne
{

extern double inward_distributed_center_size;
extern int max_bead_count;
extern coord_t default_transition_length;

enum class StrategyType
{
    Naive,
    NaiveStrategy,
    Constant,
    Center,
    Distributed,
    InwardDistributed,
    LimitedDistributed,
    SingleBead,
    OutlineAccuracy,
    COUNT
};

StrategyType toStrategyType(char c);

std::string to_string(StrategyType type);

class BeadingStrategyHelper
{
public:
    static BeadingStrategy* makeStrategy(StrategyType type, coord_t prefered_bead_width = MM2INT(0.5), float transitioning_angle = M_PI / 4, std::optional<coord_t> min_bead_width = NULL, std::optional<coord_t> min_feature_size = NULL);
};

} // namespace arachne
#endif // BEADING_STRATEGY_HELPER_H
