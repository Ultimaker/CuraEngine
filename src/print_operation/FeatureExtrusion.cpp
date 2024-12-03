// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/FeatureExtrusion.h"

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/ExtrusionMove.h"

namespace cura
{

FeatureExtrusion::FeatureExtrusion(const GCodePathConfig& config)
    : config_(config)
{
}

void FeatureExtrusion::appendExtruderMoveSequence(const ContinuousExtruderMoveSequencePtr& extruder_move_sequence, bool check_non_empty)
{
    if (! check_non_empty || ! extruder_move_sequence->empty())
    {
        appendOperation(extruder_move_sequence);
    }
}

const Velocity& FeatureExtrusion::getSpeed() const
{
    return config_.getSpeed();
}

PrintFeatureType FeatureExtrusion::getPrintFeatureType() const
{
    return config_.getPrintFeatureType();
}

coord_t FeatureExtrusion::getLineWidth() const
{
    return std::llrint(getFlow() * getWidthFactor() * static_cast<double>(config_.getLineWidth()) * config_.getFlowRatio());
}

double FeatureExtrusion::getExtrusionMM3perMM() const
{
    return config_.getExtrusionMM3perMM();
}

const Ratio& FeatureExtrusion::getFlow() const
{
    return flow_;
}

const Ratio& FeatureExtrusion::getWidthFactor() const
{
    return width_factor_;
}

} // namespace cura
