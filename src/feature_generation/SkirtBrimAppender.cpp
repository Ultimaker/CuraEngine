// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/SkirtBrimAppender.h"

#include <Application.h>
#include <Slice.h>
#include <geometry/Shape.h>
#include <settings/EnumSettings.h>
#include <settings/Settings.h>
#include <utils/SVG.h>

#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/LayerPlanPtr.h"
#include "print_operation/PrintOperationPtr.h"

namespace cura
{

void SkirtBrimAppender::process(PrintPlan* print_plan)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;
    const auto adhesion_type = settings.get<EPlatformAdhesion>("adhesion_type");
    const bool support_brim_enable = settings.get<bool>("support_brim_enable");

    if (adhesion_type != EPlatformAdhesion::SKIRT && adhesion_type != EPlatformAdhesion::BRIM && ! support_brim_enable)
    {
        return;
    }

    // Collect actually used extruders
    std::set<size_t> used_extruders;
    print_plan->findOperation(
        [&used_extruders](const PrintOperationPtr& operation)
        {
            if (const auto extruder_plan = std::dynamic_pointer_cast<ExtruderPlan>(operation))
            {
                used_extruders.insert(extruder_plan->getExtruderNr());
            }
            return false; // Loop through all operations, we don't actually search something specific
        },
        PrintOperationSequence::SearchOrder::Forward,
        1);

    // Find the maximum height we are going to print the skirt/brim to
    size_t height = 1;
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        for (const auto& extruder : Application::getInstance().current_slice_->scene.extruders)
        {
            if (used_extruders.contains(extruder.extruder_nr_))
            {
                height = std::max(height, extruder.settings_.get<size_t>("skirt_height"));
            }
        }
    }

    // Calculate the united footprint of all the extrusion features on the first layers
    std::vector<Shape> features_footprints;
    for (const LayerPlanPtr &layer_plan : print_plan->getOperationsAs<LayerPlan>())
    {
        if (layer_plan->getLayerIndex() < height)
        {
            for (const FeatureExtrusionPtr &feature_extrusion : layer_plan->findOperationsByType<FeatureExtrusion>(PrintOperationSequence::SearchOrder::Forward, 1))
            {
                features_footprints.push_back(feature_extrusion->calculateFootprint());
            }
        }
    }

    Shape footprint = Shape::unionShapes(features_footprints).getOutsidePolygons();
}

} // namespace cura