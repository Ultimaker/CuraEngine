// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/FeatureExtrusion.h"

#include "path_planning/ExtruderMoveSequence.h"
#include "path_planning/ExtruderPlan.h"
#include "path_planning/ExtrusionMove.h"
#include "path_processing/AddTravelMovesProcessor.h"

namespace cura
{

FeatureExtrusion::FeatureExtrusion(const GCodePathConfig& config)
    : config_(config)
{
}

void FeatureExtrusion::appendExtruderMoveSequence(const std::shared_ptr<ExtruderMoveSequence>& extruder_move_sequence, bool check_non_empty)
{
    if (! check_non_empty || ! extruder_move_sequence->empty())
    {
        appendOperation(extruder_move_sequence);
    }
}

void FeatureExtrusion::applyProcessors(const std::vector<const PrintOperation*>& parents)
{
    PrintOperationSequence::applyProcessors(parents);

    if (const auto extruder_plan = findParent<ExtruderPlan>(parents))
    {
        AddTravelMovesProcessor<FeatureExtrusion, ExtruderMoveSequence> add_travel_moves_processor(extruder_plan->getTravelSpeed());
        add_travel_moves_processor.process(this);
    }
}
std::optional<Point3LL> FeatureExtrusion::findStartPosition() const
{
    if (const auto first_move_sequence = findOperationByType<ExtruderMoveSequence>(SearchOrder::Forward))
    {
        return first_move_sequence->getStartPosition();
    }

    return std::nullopt;
}

std::optional<Point3LL> FeatureExtrusion::findEndPosition() const
{
    if (const auto last_move_sequence = findOperationByType<ExtruderMoveSequence>(SearchOrder::Backward))
    {
        return last_move_sequence->findEndPosition();
    }

    return std::nullopt;
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
