// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ExtrusionMove.h"

#include <spdlog/spdlog.h>

#include "plan_export/PlanExporter.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"

namespace cura
{

ExtrusionMove::ExtrusionMove(const Point3LL& position, const coord_t line_width_start, const std::optional<coord_t>& line_width_end)
    : ExtruderMove(position)
    , line_width_start_(line_width_start)
    , line_width_end_(line_width_end.value_or(line_width_start_))
{
}

void ExtrusionMove::write(PlanExporter& exporter) const
{
    const auto extruder_move_sequence = findParentByType<ContinuousExtruderMoveSequence>();
    const auto feature_extrusion = findParentByType<FeatureExtrusion>();
    const auto layer_plan = findParentByType<LayerPlan>();
    const auto extruder_plan = findParentByType<ExtruderPlan>();

    if (! feature_extrusion || ! layer_plan || ! extruder_move_sequence || ! extruder_plan)
    {
        return;
    }

    const Point3LL position = layer_plan->getAbsolutePosition(*extruder_move_sequence, getPosition());
    const Velocity velocity = feature_extrusion->getSpeed() * extruder_move_sequence->getSpeedFactor() * extruder_move_sequence->getSpeedBackPressureFactor();
    const size_t extruder_nr = extruder_plan->getExtruderNr();
    const double extrusion_mm3_per_mm = feature_extrusion->getExtrusionMM3perMM();
    const coord_t line_width = line_width_start_;
#warning add line width end
    const coord_t line_thickness = layer_plan->getThickness() + extruder_move_sequence->getZOffset() + getPosition().z_;
    const PrintFeatureType print_feature_type = feature_extrusion->getPrintFeatureType();

    exporter.writeExtrusion(position, velocity, extruder_nr, extrusion_mm3_per_mm, line_width, line_thickness, print_feature_type, false);
}

} // namespace cura
