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

StrategyType toStrategyType(char c)
{
    switch (c)
    {
        case 'n':
            return StrategyType::Naive;
        case 'N':
            return StrategyType::NaiveStrategy;
        case 'c':
            return StrategyType::Constant;
        case 'r':
            return StrategyType::Center;
        case 'd':
            return StrategyType::Distributed;
        case 'i':
            return StrategyType::InwardDistributed;
        case 's':
            return StrategyType::SingleBead;
        case 'o':
            return StrategyType::OutlineAccuracy;
    }
    return StrategyType::COUNT;
}

std::string to_string(StrategyType type)
{
    switch (type)
    {
        case StrategyType::Naive: return "Naive";
        case StrategyType::NaiveStrategy: return "NaiveStrategy";
        case StrategyType::Constant: return "Constant";
        case StrategyType::Center: return "Center";
        case StrategyType::Distributed: return "Distributed";
        case StrategyType::InwardDistributed: return "InwardDistributed";
        case StrategyType::LimitedDistributed: return "LimitedDistributed";
        case StrategyType::SingleBead: return "SingleBead";
        case StrategyType::OutlineAccuracy: return "OutlineAccuracy";
        default: return "unknown_strategy";
    }
}

class BeadingStrategyHelper
{
public:
    static BeadingStrategy* makeStrategy(StrategyType type, coord_t prefered_bead_width = MM2INT(0.5), float transitioning_angle = M_PI / 4, std::optional<coord_t> min_bead_width = NULL, std::optional<coord_t> min_feature_size = NULL)
    {
        BeadingStrategy* ret = nullptr;
        switch (type)
        {
            case StrategyType::NaiveStrategy:      ret = new NaiveBeadingStrategy(prefered_bead_width);                                      break;
            case StrategyType::Constant:           ret = new ConstantBeadingStrategy(prefered_bead_width, 4, .99 * M_PI);                    break;
            case StrategyType::Center:             ret = new CenterDeviationBeadingStrategy(prefered_bead_width, transitioning_angle);       break;
            case StrategyType::Distributed:        ret = new DistributedBeadingStrategy(prefered_bead_width, transitioning_angle);           break;
            case StrategyType::InwardDistributed:  ret = new InwardDistributedBeadingStrategy(prefered_bead_width, transitioning_angle, 2);  break;
            case StrategyType::LimitedDistributed: ret = new LimitedDistributedBeadingStrategy(prefered_bead_width, 6, transitioning_angle); break;
            case StrategyType::SingleBead:         ret = new SingleBeadBeadingStrategy(prefered_bead_width, transitioning_angle);            break;
            case StrategyType::OutlineAccuracy:    ret = new LimitedBeadingStrategy(6, new OutlineAccuracyBeadingStrategy(prefered_bead_width, prefered_bead_width * 3 / 4, prefered_bead_width / 2, transitioning_angle)); break;
            default:
                logError("Cannot make strategy!\n");
                return nullptr;
        }
        if (min_bead_width || min_feature_size)
        {
            return new WideningBeadingStrategy(ret, min_feature_size.value_or(*min_bead_width), min_bead_width.value_or(*min_feature_size));
        }
        else
        {
            return ret;
        }
    }
};

} // namespace arachne
#endif // BEADING_STRATEGY_HELPER_H
