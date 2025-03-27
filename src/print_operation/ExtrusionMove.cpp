// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ExtrusionMove.h"

#include <spdlog/spdlog.h>

#include "plan_export/PlanExporter.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"

namespace cura
{

ExtrusionMove::ExtrusionMove(const Point3LL& position, const coord_t line_width_start, const Velocity& speed, const std::optional<coord_t>& line_width_end)
    : ExtruderMove(position)
    , line_width_start_(line_width_start)
    , line_width_end_(line_width_end.value_or(line_width_start_))
    , speed_(speed)
{
}

void ExtrusionMove::write(PlanExporter& exporter) const
{
    const auto feature_extrusion = findParentByType<FeatureExtrusion>();
    const auto layer_plan = findParentByType<LayerPlan>();
    const auto extruder_plan = findParentByType<ExtruderPlan>();

    if (! feature_extrusion || ! layer_plan || ! extruder_plan)
    {
        spdlog::error("ExtrusionMove is missing a mandatory parent");
        return;
    }

    const Point3LL position = getAbsolutePosition(layer_plan);
    const Velocity& velocity = speed_;
    const size_t extruder_nr = extruder_plan->getExtruderNr();
    const coord_t line_width = line_width_start_;
    const coord_t line_thickness = layer_plan->getThickness() + getPosition().z_;
    const double extrusion_mm3_per_mm = INT2MM(line_width_start_) * INT2MM(line_thickness) * double(flow_ratio_);
#warning add line width end and extrusion_volume_end
    const PrintFeatureType print_feature_type = feature_extrusion->getType();

    exporter.writeExtrusion(position, velocity, extruder_nr, extrusion_mm3_per_mm, line_width, line_thickness, print_feature_type, false);
}

} // namespace cura
