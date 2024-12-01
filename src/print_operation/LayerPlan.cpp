// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/LayerPlan.h"

#include "plan_export/PlanExporter.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"

namespace cura
{

LayerPlan::LayerPlan(const LayerIndex& layer_index, const coord_t z, const coord_t thickness)
    : layer_index_(layer_index)
    , z_(z)
    , thickness_(thickness)
{
}

coord_t LayerPlan::getZ() const
{
    return z_;
}

LayerIndex LayerPlan::getLayerIndex() const
{
    return layer_index_;
}

void LayerPlan::appendExtruderPlan(const ExtruderPlanPtr& extruder_plan, const bool check_non_empty)
{
    if (! check_non_empty || ! extruder_plan->empty())
    {
        appendOperation(extruder_plan);
    }
}

void LayerPlan::write(PlanExporter& exporter) const
{
    const std::optional<Point3LL> start_position = findExtruderStartPosition();
    exporter.writeLayerStart(layer_index_, start_position.value_or(Point3LL()));

    PrintOperationSequence::write(exporter);

    exporter.writeLayerEnd(layer_index_, z_, thickness_);
}

std::optional<Point3LL> LayerPlan::findExtruderStartPosition() const
{
    if (const auto extruder_move_sequence = findOperationByType<ContinuousExtruderMoveSequence>(SearchOrder::DepthFirstForward))
    {
        std::optional<Point3LL> start_position = extruder_move_sequence->findStartPosition();
        if (start_position.has_value())
        {
            return getAbsolutePosition(*extruder_move_sequence, start_position.value());
        }
    }

    return std::nullopt;
}

} // namespace cura
