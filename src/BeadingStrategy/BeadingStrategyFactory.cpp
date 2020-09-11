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
        case 'r':
            return StrategyType::Center;
        case 'd':
            return StrategyType::Distributed;
        case 'i':
            return StrategyType::InwardDistributed;
    }
    return StrategyType::COUNT;
}

std::string to_string(StrategyType type)
{
    switch (type)
    {
        case StrategyType::Center:             return "CenterDeviation";
        case StrategyType::Distributed:        return "Distributed";
        case StrategyType::InwardDistributed:  return "InwardDistributed";
        default: return "unknown_strategy";
    }
}

BeadingStrategy* BeadingStrategyFactory::makeStrategy(StrategyType type, coord_t preferred_bead_width_outer, coord_t preferred_bead_width_inner, coord_t preferred_transition_length, float transitioning_angle, const coord_t* min_bead_width, const coord_t* min_feature_size, int max_bead_count)
{
    BeadingStrategy* ret = nullptr;
    switch (type)
    {
        case StrategyType::Center:             ret = new CenterDeviationBeadingStrategy(preferred_bead_width_outer, preferred_bead_width_inner, transitioning_angle);       break;
        case StrategyType::Distributed:        ret = new DistributedBeadingStrategy(preferred_bead_width_outer, preferred_bead_width_inner, preferred_transition_length, transitioning_angle);     break;
        case StrategyType::InwardDistributed:  ret = new InwardDistributedBeadingStrategy(preferred_bead_width_outer, preferred_bead_width_inner, preferred_transition_length, transitioning_angle, inward_distributed_center_size);  break;
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
