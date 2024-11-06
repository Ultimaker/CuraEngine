// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/FeatureExtrusion.h"

#include "path_planning/ExtruderMoveSequence.h"
#include "path_planning/ExtrusionMove.h"

namespace cura
{

FeatureExtrusion::FeatureExtrusion(const GCodePathConfig& config, const Point3LL& start_position)
    : config_(config)
    , start_position_(start_position)
{
}

void FeatureExtrusion::appendExtruderMoveSequence(const std::shared_ptr<ExtruderMoveSequence>& extruder_move_sequence)
{
    appendOperation(extruder_move_sequence);
}

const Point3LL& FeatureExtrusion::getStartPosition()
{
    return start_position_;
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

coord_t FeatureExtrusion::getLayerThickness() const
{
    return config_.getLayerThickness();
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
