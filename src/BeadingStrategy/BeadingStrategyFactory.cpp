//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BeadingStrategyFactory.h"

namespace cura
{

double inward_distributed_center_size = 2;

StrategyType toStrategyType(char c)
{
    switch (c)
    {
        case 'n':
            return StrategyType::Naive;
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
        case StrategyType::Naive:              return "Naive";
        case StrategyType::Center:             return "CenterDeviation";
        case StrategyType::Distributed:        return "Distributed";
        case StrategyType::InwardDistributed:  return "InwardDistributed";
        case StrategyType::LimitedDistributed: return "LimitedDistributed";  //TODO: Remove it as it is the same as Distributed + Limited meta-strategy
        default: return "unknown_strategy";
    }
}

BeadingStrategy* BeadingStrategyFactory::makeStrategy(StrategyType type, coord_t preferred_bead_width, coord_t preferred_transition_length, float transitioning_angle, const std::unique_ptr<coord_t>& min_bead_width, const std::unique_ptr<coord_t>& min_feature_size,  const coord_t max_bead_count)
{
    BeadingStrategy* ret = nullptr;
    switch (type)
    {
        case StrategyType::Naive:              ret = new NaiveBeadingStrategy(preferred_bead_width);                                      break;
        case StrategyType::Center:             ret = new CenterDeviationBeadingStrategy(preferred_bead_width, transitioning_angle);       break;
        case StrategyType::Distributed:        ret = new DistributedBeadingStrategy(preferred_bead_width, preferred_transition_length, transitioning_angle);     break;
        case StrategyType::InwardDistributed:  ret = new InwardDistributedBeadingStrategy(preferred_bead_width, preferred_transition_length, transitioning_angle, inward_distributed_center_size);  break;
        case StrategyType::LimitedDistributed: ret = new LimitedDistributedBeadingStrategy(preferred_bead_width, preferred_transition_length, max_bead_count, transitioning_angle); break;
        default:
            logError("Cannot make strategy!\n");
            return nullptr;
    }
    
    if (min_bead_width || min_feature_size)
    {
        const coord_t min_input_width = min_feature_size ? *min_feature_size : *min_bead_width;
        const coord_t min_output_width = min_bead_width ? *min_bead_width : *min_feature_size;
        logDebug("Applying the Widening Beading meta-strategy with minimum input width %d and minimum output width %d.", min_input_width, min_output_width);
        ret = new WideningBeadingStrategy(ret, min_input_width, min_output_width);
    }
    if (max_bead_count > 0)
    {
        logDebug("Applying the Limited Beading meta-strategy with maximum bead count = %d.", max_bead_count);
        ret = new LimitedBeadingStrategy(max_bead_count, ret);
    }
    return ret;
}
} // namespace cura
