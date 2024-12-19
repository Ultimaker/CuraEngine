// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshSkinGenerator.h"

#include "ExtruderTrain.h"
#include "bridge.h"
#include "infill.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/InfillFeatureExtrusion.h"
#include "settings/EnumSettings.h"
#include "settings/PathConfigStorage.h"
#include "sliceDataStorage.h"


namespace cura
{

MeshSkinGenerator::MeshSkinGenerator(const std::shared_ptr<SliceMeshStorage>& mesh)
    : MeshFeatureGenerator(mesh)
{
}

void MeshSkinGenerator::generateFeatures(
    const SliceDataStorage& storage,
    const LayerPlanPtr& layer_plan,
    const std::vector<ExtruderPlanPtr>& extruder_plans,
    const SliceLayerPart& part) const
{
    const size_t skin_extruder_nr = getMesh()->settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr_;
    const ExtruderPlanPtr extruder_plan_skin = ExtruderPlan::find(extruder_plans, skin_extruder_nr);
    assert(extruder_plan_skin && "Unable to find extruder plan for skin");

    const size_t roofing_extruder_nr = getMesh()->settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr_;
    const ExtruderPlanPtr extruder_plan_roof = ExtruderPlan::find(extruder_plans, roofing_extruder_nr);
    assert(extruder_plan_roof && "Unable to find extruder plan for roof");

    for (const SkinPart& skin_part : part.skin_parts)
    {
        processRoofing(layer_plan, skin_part, extruder_plan_roof);
        processTopBottom(storage, layer_plan, skin_part, extruder_plan_skin);
    }
}

void MeshSkinGenerator::processRoofing(const LayerPlanPtr& layer_plan, const SkinPart& skin_part, const ExtruderPlanPtr& extruder_plan) const
{
    const EFillMethod pattern = getMesh()->settings.get<EFillMethod>("roofing_pattern");
    AngleDegrees roofing_angle = 45;
    if (getMesh()->roofing_angles.size() > 0)
    {
        roofing_angle = getMesh()->roofing_angles.at(layer_plan->getLayerIndex() % getMesh()->roofing_angles.size());
    }

    const Ratio skin_density = 1.0;
    const MeshPathConfigs& mesh_configs = layer_plan->getConfigsStorage()->mesh_configs.at(getMesh());
    processSkinPrintFeature(layer_plan, skin_part.roofing_fill, mesh_configs.roofing_config, pattern, roofing_angle, skin_density, PrintFeatureType::Roof, extruder_plan);
}

void MeshSkinGenerator::processTopBottom(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const SkinPart& skin_part, const ExtruderPlanPtr& extruder_plan) const
{
    if (skin_part.skin_fill.empty())
    {
        return; // bridgeAngle requires a non-empty skin_fill.
    }

    const Settings& settings = getMesh()->settings;
    const size_t layer_nr = layer_plan->getLayerIndex();

    EFillMethod pattern = (layer_nr == 0) ? settings.get<EFillMethod>("top_bottom_pattern_0") : settings.get<EFillMethod>("top_bottom_pattern");

    AngleDegrees skin_angle = 45;
    if (getMesh()->skin_angles.size() > 0)
    {
        skin_angle = getMesh()->skin_angles.at(layer_nr % getMesh()->skin_angles.size());
    }

    // generate skin_polygons and skin_lines
    const MeshPathConfigs& mesh_configs = layer_plan->getConfigsStorage()->mesh_configs.at(getMesh());
    const GCodePathConfig* skin_config = &mesh_configs.skin_config;
    Ratio skin_density = 1.0;
    const bool bridge_settings_enabled = settings.get<bool>("bridge_settings_enabled");
    const bool bridge_enable_more_layers = bridge_settings_enabled && settings.get<bool>("bridge_enable_more_layers");
    const Ratio support_threshold = bridge_settings_enabled ? settings.get<Ratio>("bridge_skin_support_threshold") : 0.0_r;
    const size_t bottom_layers = settings.get<size_t>("bottom_layers");

    // if support is enabled, consider the support outlines so we don't generate bridges over support

    int support_layer_nr = -1;
    const SupportLayer* support_layer = nullptr;

    if (settings.get<bool>("support_enable"))
    {
        const coord_t layer_height = layer_plan->getThickness();
        const coord_t z_distance_top = settings.get<coord_t>("support_top_distance");
        const size_t z_distance_top_layers = (z_distance_top / layer_height) + 1;
        support_layer_nr = layer_nr - z_distance_top_layers;
    }

    // helper function that detects skin regions that have no support and modifies their print settings (config, line angle, density, etc.)

    auto handle_bridge_skin = [&](const int bridge_layer, const GCodePathConfig* config, const double density) // bridge_layer = 1, 2 or 3
    {
        if (support_layer_nr >= (bridge_layer - 1))
        {
            support_layer = &storage.support.supportLayers[support_layer_nr - (bridge_layer - 1)];
        }

        Shape supported_skin_part_regions;

        const double angle = bridgeAngle(settings, skin_part.skin_fill, storage, layer_nr, bridge_layer, support_layer, supported_skin_part_regions);

        if (angle > -1 || (support_threshold > 0 && (supported_skin_part_regions.area() / (skin_part.skin_fill.area() + 1) < support_threshold)))
        {
            if (angle > -1)
            {
                switch (bridge_layer)
                {
                default:
                case 1:
                    skin_angle = angle;
                    break;

                case 2:
                    if (bottom_layers > 2)
                    {
                        // orientate second bridge skin at +45 deg to first
                        skin_angle = angle + 45;
                    }
                    else
                    {
                        // orientate second bridge skin at 90 deg to first
                        skin_angle = angle + 90;
                    }
                    break;

                case 3:
                    // orientate third bridge skin at 135 (same result as -45) deg to first
                    skin_angle = angle + 135;
                    break;
                }
            }
            pattern = EFillMethod::LINES; // force lines pattern when bridging
            if (bridge_settings_enabled)
            {
                skin_config = config;
                skin_density = density;
            }
            return true;
        }

        return false;
    };

    bool is_bridge_skin = false;
    if (layer_nr > 0)
    {
        is_bridge_skin = handle_bridge_skin(1, &mesh_configs.bridge_skin_config, settings.get<Ratio>("bridge_skin_density"));
    }
    if (bridge_enable_more_layers && ! is_bridge_skin && layer_nr > 1 && bottom_layers > 1)
    {
        is_bridge_skin = handle_bridge_skin(2, &mesh_configs.bridge_skin_config2, settings.get<Ratio>("bridge_skin_density_2"));

        if (! is_bridge_skin && layer_nr > 2 && bottom_layers > 2)
        {
            is_bridge_skin = handle_bridge_skin(3, &mesh_configs.bridge_skin_config3, settings.get<Ratio>("bridge_skin_density_3"));
        }
    }

    double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;

    if (layer_nr > 0 && skin_config == &mesh_configs.skin_config && support_layer_nr >= 0 && settings.get<bool>("support_fan_enable"))
    {
        // skin isn't a bridge but is it above support and we need to modify the fan speed?

        AABB skin_bb(skin_part.skin_fill);

        support_layer = &storage.support.supportLayers[support_layer_nr];

        bool supported = false;

        if (! support_layer->support_roof.empty())
        {
            AABB support_roof_bb(support_layer->support_roof);
            if (skin_bb.hit(support_roof_bb))
            {
                supported = ! skin_part.skin_fill.intersection(support_layer->support_roof).empty();
            }
        }
        else
        {
            for (auto support_part : support_layer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (skin_bb.hit(support_part_bb))
                {
                    supported = ! skin_part.skin_fill.intersection(support_part.getInfillArea()).empty();

                    if (supported)
                    {
                        break;
                    }
                }
            }
        }

        if (supported)
        {
            fan_speed = settings.get<Ratio>("support_supported_skin_fan_speed") * 100.0;
        }
    }
    processSkinPrintFeature(layer_plan, skin_part.skin_fill, *skin_config, pattern, skin_angle, skin_density, PrintFeatureType::Skin, extruder_plan);
}

void MeshSkinGenerator::processSkinPrintFeature(
    const LayerPlanPtr& layer_plan,
    const Shape& area,
    const GCodePathConfig& config,
    EFillMethod pattern,
    const AngleDegrees skin_angle,
    const Ratio skin_density,
    const PrintFeatureType feature_type,
    const ExtruderPlanPtr& extruder_plan) const
{
    Infill::GeneratedPatterns patterns;

    const Settings& settings = getMesh()->settings;
    constexpr int infill_multiplier = 1;
    constexpr int extra_infill_shift = 0;
    const size_t wall_line_count = settings.get<size_t>("skin_outline_count");
    const coord_t small_area_width = settings.get<coord_t>("small_skin_width");
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = settings.get<bool>("connect_skin_polygons");
    coord_t max_resolution = settings.get<coord_t>("meshfix_maximum_resolution");
    coord_t max_deviation = settings.get<coord_t>("meshfix_maximum_deviation");
    const Point2LL infill_origin;
#warning to be confirmed
    // const bool skip_line_stitching = monotonic;
    constexpr bool skip_line_stitching = true;
    constexpr bool fill_gaps = true;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const bool small_areas_on_surface = settings.get<bool>("small_skin_on_surface");
    const auto& current_layer = getMesh()->layers[layer_plan->getLayerIndex()];
    const auto& exposed_to_air = current_layer.top_surface.areas.unionPolygons(current_layer.bottom_surface);
    const coord_t skin_overlap = 0; // skinfill already expanded over the roofing areas; don't overlap with perimeters

    Infill infill_comp(
        pattern,
        zig_zaggify_infill,
        connect_polygons,
        area,
        config.getLineWidth(),
        config.getLineWidth() / skin_density,
        skin_overlap,
        infill_multiplier,
        skin_angle,
        layer_plan->getZ(),
        extra_infill_shift,
        max_resolution,
        max_deviation,
        wall_line_count,
        small_area_width,
        infill_origin,
        skip_line_stitching,
        fill_gaps,
        connected_zigzags,
        use_endpieces,
        skip_some_zags,
        zag_skip_count,
        pocket_size);
    infill_comp.generate(
        patterns,
        settings,
        layer_plan->getLayerIndex(),
        SectionType::SKIN,
        nullptr,
        nullptr,
        nullptr,
        small_areas_on_surface ? Shape() : exposed_to_air);

    auto feature_extrusion = InfillFeatureExtrusion::makeFrom(patterns, feature_type, config.getLineWidth(), getMesh(), skin_angle, config.getSpeed());
    extruder_plan->appendFeatureExtrusion(feature_extrusion);
}

} // namespace cura