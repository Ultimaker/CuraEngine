// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/DraftShieldAppender.h"

#include <PrintFeatureType.h>
#include <settings/EnumSettings.h>
#include <settings/PathConfigStorage.h>
#include <utils/Simplify.h>

#include "Application.h"
#include "Slice.h"
#include "geometry/Shape.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"

namespace cura
{

DraftShieldAppender::DraftShieldAppender()
    : PrintOperationTransformer()
{
}

void DraftShieldAppender::process(PrintPlan* print_plan)
{
    const Settings& settings = MeshGroup::getCurrent()->settings;
    if (! settings.get<bool>("draft_shield_enabled"))
    {
        return;
    }

    const std::optional<LayerIndex> end_layer = calculateMaxLayer(print_plan, settings);
    constexpr std::optional<LayerIndex> start_layer = std::nullopt;
    Shape draft_shield = print_plan->calculateTotalFootprint(start_layer, end_layer, PrintFeatureTypes::model | PrintFeatureTypes::support);

    const coord_t draft_shield_dist = settings.get<coord_t>("draft_shield_dist");
    draft_shield = draft_shield.approxConvexHull(draft_shield_dist);

    // Extra offset has rounded joints, so simplify again.
    coord_t maximum_resolution = 0; // Draft shield is printed with every extruder, so resolve with the max() or min() of them to meet the requirements of all extruders.
    coord_t maximum_deviation = std::numeric_limits<coord_t>::max();
    for (const ExtruderNumber extruder_nr : print_plan->calculateUsedExtruders())
    {
        const Settings& extruder_settings = Application::getInstance().current_slice_->scene.getExtruder(extruder_nr).settings_;
        maximum_resolution = std::max(maximum_resolution, extruder_settings.get<coord_t>("meshfix_maximum_resolution"));
        maximum_deviation = std::min(maximum_deviation, extruder_settings.get<coord_t>("meshfix_maximum_deviation"));
    }

    draft_shield = Simplify(maximum_resolution, maximum_deviation, 0).polygon(draft_shield);
    // if (storage.prime_tower_)
    // {
    //     coord_t max_line_width = 0;
    //     { // compute max_line_width
    //         const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    //         const auto& extruders = Application::getInstance().current_slice_->scene.extruders;
    //         for (int extruder_nr = 0; extruder_nr < int(extruders.size()); extruder_nr++)
    //         {
    //             if (! extruder_is_used[extruder_nr])
    //                 continue;
    //             max_line_width = std::max(max_line_width, extruders[extruder_nr].settings_.get<coord_t>("skirt_brim_line_width"));
    //         }
    //     }
    //     storage.draft_protection_shield = storage.draft_protection_shield.difference(storage.prime_tower_->getOccupiedGroundOutline().offset(max_line_width / 2));
    // }


    // const LayerIndex layer_nr = std::max(LayerIndex{ 0 }, gcode_layer.getLayerIndex());
    if (draft_shield.empty())
    {
        return;
    }

    for (LayerPlanPtr layer_plan : print_plan->getOperationsAs<LayerPlan>())
    {
        ExtruderPlanPtr first_extruder_plan = layer_plan->findFirstExtruderPlan();
        if (first_extruder_plan)
        {
            const ExtruderNumber extruder_nr = first_extruder_plan->getExtruderNr();
            const coord_t line_width = Scene::getCurrent().getExtruder(extruder_nr).settings_.getLineWidth(layer_plan->getLayerIndex());
            const GCodePathConfig& config = layer_plan->getConfigsStorage()->skirt_brim_config_per_extruder.at(extruder_nr);

            auto feature_extrusion = std::make_shared<FeatureExtrusion>(PrintFeatureType::DraftShield, line_width);
            for (const Polygon& polygon : draft_shield)
            {
                feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(polygon, line_width, config.getSpeed()));
            }
            first_extruder_plan->appendFeatureExtrusion(feature_extrusion);
        }
    }

    // if (mesh_group_settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation") == DraftShieldHeightLimitation::LIMITED)
    // {
    //     const coord_t draft_shield_height = mesh_group_settings.get<coord_t>("draft_shield_height");
    //     const coord_t layer_height_0 = mesh_group_settings.get<coord_t>("layer_height_0");
    //     const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    //     const LayerIndex max_screen_layer = (draft_shield_height - layer_height_0) / layer_height + 1;
    //     if (layer_nr > max_screen_layer)
    //     {
    //         return;
    //     }
    // }
    //
    // gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, gcode_layer.configs_storage_.skirt_brim_config_per_extruder[0], mesh_group_settings);
}

std::optional<LayerIndex> DraftShieldAppender::calculateMaxLayer(const PrintPlan* print_plan, const Settings& settings)
{
    std::optional<LayerIndex> max_layer;

    switch (settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation"))
    {
    case DraftShieldHeightLimitation::FULL:
        break;
    case DraftShieldHeightLimitation::LIMITED:
    {
        const auto draft_shield_height = settings.get<coord_t>("draft_shield_height");
        if (const LayerPlanPtr layer_plan = print_plan->findLayerPlanAtHeight(draft_shield_height))
        {
            max_layer = layer_plan->getLayerIndex();
        }
        break;
    }
    case DraftShieldHeightLimitation::PLUGIN:
        spdlog::error("Generating draft shield height using a plugin is not implemented yet");
        break;
    }

    return max_layer;
}

} // namespace cura