// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/LayerPlan.h"

#include "plan_export/PlanExporter.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderChange.h"
#include "print_operation/ExtruderPlan.h"

namespace cura
{

LayerPlan::LayerPlan(const LayerIndex& layer_index, const coord_t z, const coord_t thickness, const std::shared_ptr<PathConfigStorage>& configs)
    : layer_index_(layer_index)
    , z_(z)
    , thickness_(thickness)
    , configs_(configs)
{
}

coord_t LayerPlan::getZ() const
{
    return z_;
}

coord_t LayerPlan::getThickness() const
{
    return thickness_;
}

const std::shared_ptr<PathConfigStorage>& LayerPlan::getConfigsStorage() const
{
    return configs_;
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

void LayerPlan::insertExtruderChangeAfter(const ExtruderPlanPtr& extruder_plan, const std::shared_ptr<ExtruderChange>& extruder_change)
{
    insertOperationAfter(extruder_plan, extruder_change);
}

void LayerPlan::write(PlanExporter& exporter) const
{
    const std::optional<Point3LL> start_position = findStartPosition();
    exporter.writeLayerStart(layer_index_, getAbsolutePosition(start_position.value_or(Point3LL())));

    PrintOperationSequence::write(exporter);

    exporter.writeLayerEnd(layer_index_, z_, thickness_);
}

Point3LL LayerPlan::getAbsolutePosition(const Point3LL& relative_position) const
{
    Point3LL absolute_position = relative_position;
    absolute_position.z_ += getZ();
    return absolute_position;
}

ExtruderPlanPtr LayerPlan::findFirstExtruderPlan(const ExtruderNumber& extruder_nr) const
{
    return findOperationByType<ExtruderPlan>(
        SearchOrder::Forward,
        SearchDepth::DirectChildren,
        [&extruder_nr](const ExtruderPlanPtr& extruder_plan)
        {
            return extruder_plan->getExtruderNr() == extruder_nr;
        });
}

} // namespace cura
