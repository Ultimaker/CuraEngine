// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/PathConfigStorage.h"

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "raft.h"
#include "settings/EnumSettings.h" //For EPlatformAdhesion.
#include "settings/Settings.h" // MAX_INFILL_COMBINE
#include "sliceDataStorage.h" // SliceDataStorage

namespace cura
{

std::vector<Ratio> PathConfigStorage::getLineWidthFactorPerExtruder(const LayerIndex& layer_nr)
{
    std::vector<Ratio> ret;
    for (const ExtruderTrain& train : Application::getInstance().current_slice->scene.extruders)
    {
        if (layer_nr <= 0)
        {
            const Ratio factor = train.settings.get<Ratio>("initial_layer_line_width_factor");
            ret.push_back(factor);
        }
        else
        {
            ret.push_back(1.0);
        }
    }
    return ret;
}

PathConfigStorage::PathConfigStorage(const SliceDataStorage& storage, const LayerIndex& layer_nr, const coord_t layer_thickness)
    : support_infill_extruder_nr(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr)
    , support_roof_extruder_nr(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr)
    , support_bottom_extruder_nr(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr)
    , raft_base_train(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_base_extruder_nr"))
    , raft_interface_train(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_interface_extruder_nr"))
    , raft_surface_train(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_surface_extruder_nr"))
    , support_infill_train(Application::getInstance().current_slice->scene.extruders[support_infill_extruder_nr])
    , support_roof_train(Application::getInstance().current_slice->scene.extruders[support_roof_extruder_nr])
    , support_bottom_train(Application::getInstance().current_slice->scene.extruders[support_bottom_extruder_nr])
    , line_width_factor_per_extruder(PathConfigStorage::getLineWidthFactorPerExtruder(layer_nr))
    , raft_base_config(GCodePathConfig{ .type = PrintFeatureType::SupportInterface,
                                        .line_width = raft_base_train.settings.get<coord_t>("raft_base_line_width"),
                                        .layer_thickness = raft_base_train.settings.get<coord_t>("raft_base_thickness"),
                                        .flow = Ratio(1.0),
                                        .speed_derivatives = SpeedDerivatives{ .speed = raft_base_train.settings.get<Velocity>("raft_base_speed"),
                                                                               .acceleration = raft_base_train.settings.get<Acceleration>("raft_base_acceleration"),
                                                                               .jerk = raft_base_train.settings.get<Velocity>("raft_base_jerk") } })
    , raft_interface_config(GCodePathConfig{ .type = PrintFeatureType::Support,
                                             .line_width = raft_interface_train.settings.get<coord_t>("raft_interface_line_width"),
                                             .layer_thickness = raft_interface_train.settings.get<coord_t>("raft_interface_thickness"),
                                             .flow = Ratio(1.0),
                                             .speed_derivatives = SpeedDerivatives{ .speed = raft_interface_train.settings.get<Velocity>("raft_interface_speed"),
                                                                                    .acceleration = raft_interface_train.settings.get<Acceleration>("raft_interface_acceleration"),
                                                                                    .jerk = raft_interface_train.settings.get<Velocity>("raft_interface_jerk") } })
    , raft_surface_config(GCodePathConfig{ .type = PrintFeatureType::SupportInterface,
                                           .line_width = raft_surface_train.settings.get<coord_t>("raft_surface_line_width"),
                                           .layer_thickness = raft_surface_train.settings.get<coord_t>("raft_surface_thickness"),
                                           .flow = Ratio(1.0),
                                           .speed_derivatives = SpeedDerivatives{ .speed = raft_surface_train.settings.get<Velocity>("raft_surface_speed"),
                                                                                  .acceleration = raft_surface_train.settings.get<Acceleration>("raft_surface_acceleration"),
                                                                                  .jerk = raft_surface_train.settings.get<Velocity>("raft_surface_jerk") } })
    , support_roof_config(GCodePathConfig{
          .type = PrintFeatureType::SupportInterface,
          .line_width = static_cast<coord_t>(support_roof_train.settings.get<coord_t>("support_roof_line_width") * line_width_factor_per_extruder[support_roof_extruder_nr]),
          .layer_thickness = layer_thickness,
          .flow
          = support_roof_train.settings.get<Ratio>("support_roof_material_flow") * ((layer_nr == 0) ? support_roof_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)),
          .speed_derivatives = { .speed = support_roof_train.settings.get<Velocity>("speed_support_roof"),
                                 .acceleration = support_roof_train.settings.get<Acceleration>("acceleration_support_roof"),
                                 .jerk = support_roof_train.settings.get<Velocity>("jerk_support_roof") } })
    , support_bottom_config(GCodePathConfig{
          .type = PrintFeatureType::SupportInterface,
          .line_width = static_cast<coord_t>(support_bottom_train.settings.get<coord_t>("support_bottom_line_width") * line_width_factor_per_extruder[support_bottom_extruder_nr]),
          .layer_thickness = layer_thickness,
          .flow = support_roof_train.settings.get<Ratio>("support_bottom_material_flow")
                * ((layer_nr == 0) ? support_roof_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)),
          .speed_derivatives = SpeedDerivatives{ .speed = support_bottom_train.settings.get<Velocity>("speed_support_bottom"),
                                                 .acceleration = support_bottom_train.settings.get<Acceleration>("acceleration_support_bottom"),
                                                 .jerk = support_bottom_train.settings.get<Velocity>("jerk_support_bottom") } })
{
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    travel_config_per_extruder.reserve(extruder_count);
    skirt_brim_config_per_extruder.reserve(extruder_count);
    prime_tower_config_per_extruder.reserve(extruder_count);
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
        travel_config_per_extruder.emplace_back(GCodePathConfig{ .type = PrintFeatureType::MoveCombing,
                                                                 .line_width = 0,
                                                                 .layer_thickness = 0,
                                                                 .flow = 0.0,
                                                                 .speed_derivatives = SpeedDerivatives{ .speed = train.settings.get<Velocity>("speed_travel"),
                                                                                                        .acceleration = train.settings.get<Acceleration>("acceleration_travel"),
                                                                                                        .jerk = train.settings.get<Velocity>("jerk_travel") } });
        skirt_brim_config_per_extruder.emplace_back(
            GCodePathConfig{ .type = PrintFeatureType::SkirtBrim,
                             .line_width = static_cast<coord_t>(
                                 train.settings.get<coord_t>("skirt_brim_line_width")
                                 * ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT)
                                        ? 1.0_r
                                        : line_width_factor_per_extruder[extruder_nr])) // cause it's also used for the draft/ooze shield
                             ,
                             .layer_thickness = layer_thickness,
                             .flow = train.settings.get<Ratio>("skirt_brim_material_flow") * ((layer_nr == 0) ? train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)),
                             .speed_derivatives = SpeedDerivatives{ .speed = train.settings.get<Velocity>("skirt_brim_speed"),
                                                                    .acceleration = train.settings.get<Acceleration>("acceleration_skirt_brim"),
                                                                    .jerk = train.settings.get<Velocity>("jerk_skirt_brim") } });
        prime_tower_config_per_extruder.emplace_back(GCodePathConfig{
            .type = PrintFeatureType::PrimeTower,
            .line_width = static_cast<coord_t>(
                train.settings.get<coord_t>("prime_tower_line_width")
                * ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[extruder_nr])),
            .layer_thickness = layer_thickness,
            .flow = train.settings.get<Ratio>("prime_tower_flow") * ((layer_nr == 0) ? train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)),
            .speed_derivatives = SpeedDerivatives{ .speed = train.settings.get<Velocity>("speed_prime_tower"),
                                                   .acceleration = train.settings.get<Acceleration>("acceleration_prime_tower"),
                                                   .jerk = train.settings.get<Velocity>("jerk_prime_tower") } });
    }

    mesh_configs.reserve(storage.meshes.size());
    for (const std::shared_ptr<SliceMeshStorage>& mesh_storage : storage.meshes)
    {
        mesh_configs.emplace_back(*mesh_storage, layer_thickness, layer_nr, line_width_factor_per_extruder);
    }

    support_infill_config.reserve(MAX_INFILL_COMBINE);
    const float support_infill_line_width_factor
        = (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[support_infill_extruder_nr];
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        support_infill_config.emplace_back(
            GCodePathConfig{ .type = PrintFeatureType::Support,
                             .line_width = static_cast<coord_t>(support_infill_train.settings.get<coord_t>("support_line_width") * support_infill_line_width_factor),
                             .layer_thickness = layer_thickness,
                             .flow = support_infill_train.settings.get<Ratio>("support_material_flow")
                                   * ((layer_nr == 0) ? support_infill_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)) * (combine_idx + 1),
                             .speed_derivatives = SpeedDerivatives{ .speed = support_infill_train.settings.get<Velocity>("speed_support_infill"),
                                                                    .acceleration = support_infill_train.settings.get<Acceleration>("acceleration_support_infill"),
                                                                    .jerk = support_infill_train.settings.get<Velocity>("jerk_support_infill") } });
    }

    const size_t initial_speedup_layer_count = mesh_group_settings.get<size_t>("speed_slowdown_layers");
    if (layer_nr >= 0 && static_cast<size_t>(layer_nr) < initial_speedup_layer_count)
    {
        handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
    }

    const auto layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
    const auto support_top_distance = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("support_top_distance");
    const coord_t leftover_support_distance = support_top_distance % layer_height;

    support_fractional_infill_config = support_infill_config; // copy
    for (auto& config : support_fractional_infill_config)
    {
        config.z_offset = -leftover_support_distance;
        config.flow *= Ratio(layer_height - leftover_support_distance, layer_height);
    }

    support_fractional_roof_config = support_roof_config; // copy
    support_fractional_roof_config.z_offset = -leftover_support_distance;
    support_fractional_roof_config.flow *= Ratio(layer_height - leftover_support_distance, layer_height);
}

void MeshPathConfigs::smoothAllSpeeds(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer)
{
    inset0_config.speed_derivatives.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    insetX_config.speed_derivatives.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    skin_config.speed_derivatives.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    ironing_config.speed_derivatives.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    for (size_t idx = 0; idx < MAX_INFILL_COMBINE; idx++)
    {
        // Infill speed (per combine part per mesh).
        infill_config[idx].speed_derivatives.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    }
}

void PathConfigStorage::handleInitialLayerSpeedup(const SliceDataStorage& storage, const LayerIndex& layer_nr, const size_t initial_speedup_layer_count)
{
    std::vector<SpeedDerivatives> global_first_layer_config_per_extruder;
    global_first_layer_config_per_extruder.reserve(Application::getInstance().current_slice->scene.extruders.size());
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
    {
        global_first_layer_config_per_extruder.emplace_back(SpeedDerivatives{ .speed = extruder.settings.get<Velocity>("speed_print_layer_0"),
                                                                              .acceleration = extruder.settings.get<Acceleration>("acceleration_print_layer_0"),
                                                                              .jerk = extruder.settings.get<Velocity>("jerk_print_layer_0") });
    }

    { // support
        if (layer_nr < static_cast<LayerIndex>(initial_speedup_layer_count))
        {
            const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
            const size_t extruder_nr_support_infill
                = mesh_group_settings.get<ExtruderTrain&>((layer_nr <= 0) ? "support_extruder_nr_layer_0" : "support_infill_extruder_nr").extruder_nr;
            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                support_infill_config[idx].speed_derivatives.smoothSpeed(
                    global_first_layer_config_per_extruder[extruder_nr_support_infill],
                    std::max(LayerIndex(0), layer_nr),
                    initial_speedup_layer_count);
            }
            const size_t extruder_nr_support_roof = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
            support_roof_config.speed_derivatives.smoothSpeed(
                global_first_layer_config_per_extruder[extruder_nr_support_roof],
                std::max(LayerIndex(0), layer_nr),
                initial_speedup_layer_count);
            const size_t extruder_nr_support_bottom = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
            support_bottom_config.speed_derivatives.smoothSpeed(
                global_first_layer_config_per_extruder[extruder_nr_support_bottom],
                std::max(LayerIndex(0), layer_nr),
                initial_speedup_layer_count);
        }
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
        {
            const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
            const SpeedDerivatives initial_layer_travel_speed_config{ .speed = train.settings.get<Velocity>("speed_travel_layer_0"),
                                                                      .acceleration = train.settings.get<Acceleration>("acceleration_travel_layer_0"),
                                                                      .jerk = train.settings.get<Velocity>("jerk_travel_layer_0") };
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

            travel.speed_derivatives.smoothSpeed(initial_layer_travel_speed_config, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);

            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
            prime_tower.speed_derivatives.smoothSpeed(global_first_layer_config_per_extruder[extruder_nr], std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
        }
    }

    { // meshes
        for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];

            const SpeedDerivatives initial_layer_speed_config{ .speed = mesh.settings.get<Velocity>("speed_print_layer_0"),
                                                               .acceleration = mesh.settings.get<Acceleration>("acceleration_print_layer_0"),
                                                               .jerk = mesh.settings.get<Velocity>("jerk_print_layer_0") };

            mesh_configs[mesh_idx].smoothAllSpeeds(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
            mesh_configs[mesh_idx].roofing_config.speed_derivatives.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
        }
    }
}

} // namespace cura
