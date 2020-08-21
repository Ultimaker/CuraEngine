//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategyFactory.h"

namespace cura
{

double inward_distributed_center_size = 2;
int max_bead_count = -1;
coord_t default_transition_length = 400;

StrategyType toStrategyType(char c)
{
    switch (c)
    {
        case 'n':
            return StrategyType::Naive;
        case 'N':
            return StrategyType::NaiveStrategy;
        case 'r':
            return StrategyType::Center;
        case 'd':
            return StrategyType::Distributed;
        case 'i':
            return StrategyType::InwardDistributed;
        case 'l':
            return StrategyType::LimitedDistributed;
    }
    return StrategyType::COUNT;
}

std::string to_string(StrategyType type)
{
    switch (type)
    {
        case StrategyType::Naive: return "Naive";
        case StrategyType::NaiveStrategy: return "NaiveStrategy";
        case StrategyType::Center: return "Center";
        case StrategyType::Distributed: return "Distributed";
        case StrategyType::InwardDistributed: return "InwardDistributed";
        case StrategyType::LimitedDistributed: return "LimitedDistributed";
        default: return "unknown_strategy";
    }
}

BeadingStrategy* BeadingStrategyFactory::makeStrategy(StrategyType type, coord_t prefered_bead_width, float transitioning_angle, std::optional<coord_t> min_bead_width, std::optional<coord_t> min_feature_size)
{
    BeadingStrategy* ret = nullptr;
    switch (type)
    {
        case StrategyType::NaiveStrategy:      ret = new NaiveBeadingStrategy(prefered_bead_width);                                      break;
        case StrategyType::Center:             ret = new CenterDeviationBeadingStrategy(prefered_bead_width, transitioning_angle);       break;
        case StrategyType::Distributed:        ret = new DistributedBeadingStrategy(prefered_bead_width, default_transition_length, transitioning_angle);     break;
        case StrategyType::InwardDistributed:  ret = new InwardDistributedBeadingStrategy(prefered_bead_width, default_transition_length, transitioning_angle, inward_distributed_center_size);  break;
        case StrategyType::LimitedDistributed: ret = new LimitedDistributedBeadingStrategy(prefered_bead_width, default_transition_length, max_bead_count, transitioning_angle); break;
        default:
            logError("Cannot make strategy!\n");
            return nullptr;
    }
    
    if (min_bead_width || min_feature_size)
    {
        ret = new WideningBeadingStrategy(ret, min_feature_size.value_or(*min_bead_width), min_bead_width.value_or(*min_feature_size));
    }
    if (max_bead_count > 0)
    {
        ret = new LimitedBeadingStrategy(max_bead_count, ret);
    }
    return ret;
}
} // namespace cura
