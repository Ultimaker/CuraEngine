// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtrusionMove.h"

#include <spdlog/spdlog.h>

#include "path_export/PathExporter.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_planning/LayerPlan.h"

namespace cura
{

ExtrusionMove::ExtrusionMove(const Point3LL& position, const Ratio& line_width_ratio)
    : ExtruderMove(position)
    , line_width_ratio_(line_width_ratio)
{
}

void ExtrusionMove::write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents) const
{
    auto extruder_move_set = findParent<ExtruderMoveSequence>(parents);
    auto feature_extrusion = findParent<FeatureExtrusion>(parents);
    auto layer_plan = findParent<LayerPlan>(parents);

    if (! feature_extrusion || ! layer_plan || ! extruder_move_set)
    {
        return;
    }

    const Point3LL position = getAbsolutePosition(*layer_plan, *extruder_move_set);
    const Velocity velocity = feature_extrusion->getSpeed() * feature_extrusion->getSpeedFactor() * feature_extrusion->getSpeedBackPressureFactor();
    const double extrusion_mm3_per_mm = feature_extrusion->getExtrusionMM3perMM();
    const coord_t line_width = std::llrint(feature_extrusion->getLineWidth() * static_cast<double>(line_width_ratio_));
    const coord_t line_thickness = feature_extrusion->getLayerThickness() + feature_extrusion->getZOffset() + getPosition().z_;
    const PrintFeatureType print_feature_type = feature_extrusion->getPrintFeatureType();

    exporter.writeExtrusion(position, velocity, extrusion_mm3_per_mm, line_width, line_thickness, print_feature_type, false);
}

} // namespace cura
