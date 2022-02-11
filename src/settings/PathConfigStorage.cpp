//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathConfigStorage.h"
#include "Settings.h" // MAX_INFILL_COMBINE
#include "../Application.h"
#include "../ExtruderTrain.h"
#include "../raft.h"
#include "../Slice.h"
#include "../sliceDataStorage.h" // SliceDataStorage
#include "../settings/EnumSettings.h" //For EPlatformAdhesion.

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

PathConfigStorage::MeshPathConfigs::MeshPathConfigs(const SliceMeshStorage& mesh, const coord_t layer_thickness, const LayerIndex& layer_nr, const std::vector<Ratio>& line_width_factor_per_extruder)
: inset0_config(
    PrintFeatureType::OuterWall
    , mesh.settings.get<coord_t>("wall_line_width_0") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("wall_0_material_flow") * ((layer_nr == 0) ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_wall_0"), mesh.settings.get<Acceleration>("acceleration_wall_0"), mesh.settings.get<Velocity>("jerk_wall_0")}
)
, insetX_config(
    PrintFeatureType::InnerWall
    , mesh.settings.get<coord_t>("wall_line_width_x") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("wall_x_material_flow") * ((layer_nr == 0) ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_wall_x"), mesh.settings.get<Acceleration>("acceleration_wall_x"), mesh.settings.get<Velocity>("jerk_wall_x")}
)
, bridge_inset0_config(
    PrintFeatureType::OuterWall
    , mesh.settings.get<coord_t>("wall_line_width_0") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("bridge_wall_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("bridge_wall_speed"), mesh.settings.get<Acceleration>("acceleration_wall_0"), mesh.settings.get<Velocity>("jerk_wall_0")}
    , true // is_bridge_path
    , mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0
)
, bridge_insetX_config(
    PrintFeatureType::InnerWall
    , mesh.settings.get<coord_t>("wall_line_width_x") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("bridge_wall_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("bridge_wall_speed"), mesh.settings.get<Acceleration>("acceleration_wall_x"), mesh.settings.get<Velocity>("jerk_wall_x")}
    , true // is_bridge_path
    , mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0
)
, skin_config(
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("skin_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("skin_material_flow") * ((layer_nr == 0) ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_topbottom"), mesh.settings.get<Acceleration>("acceleration_topbottom"), mesh.settings.get<Velocity>("jerk_topbottom")}
)
, bridge_skin_config( // use bridge skin flow, speed and fan
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("skin_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("bridge_skin_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("bridge_skin_speed"), mesh.settings.get<Acceleration>("acceleration_topbottom"), mesh.settings.get<Velocity>("jerk_topbottom")}
    , true // is_bridge_path
    , mesh.settings.get<Ratio>("bridge_fan_speed") * 100.0
)
, bridge_skin_config2( // use bridge skin 2 flow, speed and fan
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("skin_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("bridge_skin_material_flow_2")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("bridge_skin_speed_2"), mesh.settings.get<Acceleration>("acceleration_topbottom"), mesh.settings.get<Velocity>("jerk_topbottom")}
    , true // is_bridge_path
    , mesh.settings.get<Ratio>("bridge_fan_speed_2") * 100.0
)
, bridge_skin_config3( // use bridge skin 3 flow, speed and fan
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("skin_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr]
    , layer_thickness
    , mesh.settings.get<Ratio>("bridge_skin_material_flow_3")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("bridge_skin_speed_3"), mesh.settings.get<Acceleration>("acceleration_topbottom"), mesh.settings.get<Velocity>("jerk_topbottom")}
    , true // is_bridge_path
    , mesh.settings.get<Ratio>("bridge_fan_speed_3") * 100.0
)
, roofing_config(
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("roofing_line_width")
    , layer_thickness
    , mesh.settings.get<Ratio>("roofing_material_flow") * ((layer_nr == 0) ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_roofing"), mesh.settings.get<Acceleration>("acceleration_roofing"), mesh.settings.get<Velocity>("jerk_roofing")}
)
, ironing_config(
    PrintFeatureType::Skin
    , mesh.settings.get<coord_t>("skin_line_width")
    , layer_thickness
    , mesh.settings.get<Ratio>("ironing_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_ironing"), mesh.settings.get<Acceleration>("acceleration_ironing"), mesh.settings.get<Velocity>("jerk_ironing")}
)

{
    infill_config.reserve(MAX_INFILL_COMBINE);

    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        infill_config.emplace_back(
                PrintFeatureType::Infill
                , mesh.settings.get<coord_t>("infill_line_width") * line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr]
                , layer_thickness
                , mesh.settings.get<Ratio>("infill_material_flow") * ((layer_nr == 0) ? mesh.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)) * (combine_idx + 1)
                , GCodePathConfig::SpeedDerivatives{mesh.settings.get<Velocity>("speed_infill"), mesh.settings.get<Acceleration>("acceleration_infill"), mesh.settings.get<Velocity>("jerk_infill")}
            );
    }
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
, raft_base_config(
            PrintFeatureType::SupportInterface
            , raft_base_train.settings.get<coord_t>("raft_base_line_width")
            , raft_base_train.settings.get<coord_t>("raft_base_thickness")
            , Ratio(1.0)
            , GCodePathConfig::SpeedDerivatives{raft_base_train.settings.get<Velocity>("raft_base_speed"), raft_base_train.settings.get<Acceleration>("raft_base_acceleration"), raft_base_train.settings.get<Velocity>("raft_base_jerk")}
        )
, raft_interface_config(
            PrintFeatureType::Support
            , raft_interface_train.settings.get<coord_t>("raft_interface_line_width")
            , raft_interface_train.settings.get<coord_t>("raft_interface_thickness")
            , Ratio(1.0)
            , GCodePathConfig::SpeedDerivatives{raft_interface_train.settings.get<Velocity>("raft_interface_speed"), raft_interface_train.settings.get<Acceleration>("raft_interface_acceleration"), raft_interface_train.settings.get<Velocity>("raft_interface_jerk")}
        )
, raft_surface_config(
            PrintFeatureType::SupportInterface
            , raft_surface_train.settings.get<coord_t>("raft_surface_line_width")
            , raft_surface_train.settings.get<coord_t>("raft_surface_thickness")
            , Ratio(1.0)
            , GCodePathConfig::SpeedDerivatives{raft_surface_train.settings.get<Velocity>("raft_surface_speed"), raft_surface_train.settings.get<Acceleration>("raft_surface_acceleration"), raft_surface_train.settings.get<Velocity>("raft_surface_jerk")}
        )
, support_roof_config(
            PrintFeatureType::SupportInterface
            , support_roof_train.settings.get<coord_t>("support_roof_line_width") * line_width_factor_per_extruder[support_roof_extruder_nr]
            , layer_thickness
            , support_roof_train.settings.get<Ratio>("support_roof_material_flow") * ((layer_nr == 0) ? support_roof_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
            , GCodePathConfig::SpeedDerivatives{support_roof_train.settings.get<Velocity>("speed_support_roof"), support_roof_train.settings.get<Acceleration>("acceleration_support_roof"), support_roof_train.settings.get<Velocity>("jerk_support_roof")}
        )
, support_bottom_config(
            PrintFeatureType::SupportInterface
            , support_bottom_train.settings.get<coord_t>("support_bottom_line_width") * line_width_factor_per_extruder[support_bottom_extruder_nr]
            , layer_thickness
            , support_roof_train.settings.get<Ratio>("support_bottom_material_flow") * ((layer_nr == 0) ? support_roof_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
            , GCodePathConfig::SpeedDerivatives{support_bottom_train.settings.get<Velocity>("speed_support_bottom"), support_bottom_train.settings.get<Acceleration>("acceleration_support_bottom"), support_bottom_train.settings.get<Velocity>("jerk_support_bottom")}
        )
{
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    travel_config_per_extruder.reserve(extruder_count);
    skirt_brim_config_per_extruder.reserve(extruder_count);
    prime_tower_config_per_extruder.reserve(extruder_count);
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
        travel_config_per_extruder.emplace_back(
                PrintFeatureType::MoveCombing
                , 0
                , 0
                , 0.0
                , GCodePathConfig::SpeedDerivatives{train.settings.get<Velocity>("speed_travel"), train.settings.get<Acceleration>("acceleration_travel"), train.settings.get<Velocity>("jerk_travel")}
            );
        skirt_brim_config_per_extruder.emplace_back(
                PrintFeatureType::SkirtBrim
                , train.settings.get<coord_t>("skirt_brim_line_width")
                    * ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[extruder_nr]) // cause it's also used for the draft/ooze shield
                , layer_thickness
                , train.settings.get<Ratio>("skirt_brim_material_flow") * ((layer_nr == 0) ? train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
                , GCodePathConfig::SpeedDerivatives{train.settings.get<Velocity>("skirt_brim_speed"), train.settings.get<Acceleration>("acceleration_skirt_brim"), train.settings.get<Velocity>("jerk_skirt_brim")}
            );
        prime_tower_config_per_extruder.emplace_back(
                PrintFeatureType::PrimeTower
                , train.settings.get<coord_t>("prime_tower_line_width")
                    * ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[extruder_nr])
                , layer_thickness
                , train.settings.get<Ratio>("prime_tower_flow") * ((layer_nr == 0) ? train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0))
                , GCodePathConfig::SpeedDerivatives{train.settings.get<Velocity>("speed_prime_tower"), train.settings.get<Acceleration>("acceleration_prime_tower"), train.settings.get<Velocity>("jerk_prime_tower")}
            );
    }

    mesh_configs.reserve(storage.meshes.size());
    for (const SliceMeshStorage& mesh_storage : storage.meshes)
    {
        mesh_configs.emplace_back(mesh_storage, layer_thickness, layer_nr, line_width_factor_per_extruder);
    }

    support_infill_config.reserve(MAX_INFILL_COMBINE);
    const float support_infill_line_width_factor = (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[support_infill_extruder_nr];
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        support_infill_config.emplace_back(
            PrintFeatureType::Support
            , support_infill_train.settings.get<coord_t>("support_line_width") * support_infill_line_width_factor
            , layer_thickness
            , support_infill_train.settings.get<Ratio>("support_material_flow") * ((layer_nr == 0) ? support_infill_train.settings.get<Ratio>("material_flow_layer_0") : Ratio(1.0)) * (combine_idx + 1)
            , GCodePathConfig::SpeedDerivatives{support_infill_train.settings.get<Velocity>("speed_support_infill"), support_infill_train.settings.get<Acceleration>("acceleration_support_infill"), support_infill_train.settings.get<Velocity>("jerk_support_infill")}
        );
    }

    const size_t initial_speedup_layer_count = mesh_group_settings.get<size_t>("speed_slowdown_layers");
    if (layer_nr >= 0 && static_cast<size_t>(layer_nr) < initial_speedup_layer_count)
    {
        handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
    }
}

void PathConfigStorage::MeshPathConfigs::smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, const LayerIndex& layer_nr, const LayerIndex& max_speed_layer)
{
    inset0_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
    insetX_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
    skin_config.smoothSpeed(                first_layer_config, layer_nr, max_speed_layer);
    ironing_config.smoothSpeed(             first_layer_config, layer_nr, max_speed_layer);
    for (size_t idx = 0; idx < MAX_INFILL_COMBINE; idx++)
    {
        //Infill speed (per combine part per mesh).
        infill_config[idx].smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    }
}

void cura::PathConfigStorage::handleInitialLayerSpeedup(const SliceDataStorage& storage, const LayerIndex& layer_nr, const size_t initial_speedup_layer_count)
{
    std::vector<GCodePathConfig::SpeedDerivatives> global_first_layer_config_per_extruder;
    global_first_layer_config_per_extruder.reserve(Application::getInstance().current_slice->scene.extruders.size());
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
    {
        global_first_layer_config_per_extruder.emplace_back(
            GCodePathConfig::SpeedDerivatives{
                extruder.settings.get<Velocity>("speed_print_layer_0")
                , extruder.settings.get<Acceleration>("acceleration_print_layer_0")
                , extruder.settings.get<Velocity>("jerk_print_layer_0")
            });
    }

    { // support
        if (layer_nr < static_cast<LayerIndex>(initial_speedup_layer_count))
        {
            const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
            const size_t extruder_nr_support_infill = mesh_group_settings.get<ExtruderTrain&>((layer_nr <= 0) ? "support_extruder_nr_layer_0" : "support_infill_extruder_nr").extruder_nr;
            GCodePathConfig::SpeedDerivatives& first_layer_config_infill = global_first_layer_config_per_extruder[extruder_nr_support_infill];
            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                support_infill_config[idx].smoothSpeed(first_layer_config_infill, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
            }

            const size_t extruder_nr_support_roof = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
            GCodePathConfig::SpeedDerivatives& first_layer_config_roof = global_first_layer_config_per_extruder[extruder_nr_support_roof];
            support_roof_config.smoothSpeed(first_layer_config_roof, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
            const size_t extruder_nr_support_bottom = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
            GCodePathConfig::SpeedDerivatives& first_layer_config_bottom = global_first_layer_config_per_extruder[extruder_nr_support_bottom];
            support_bottom_config.smoothSpeed(first_layer_config_bottom, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
        }
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
        {
            const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
            GCodePathConfig::SpeedDerivatives initial_layer_travel_speed_config{
                    train.settings.get<Velocity>("speed_travel_layer_0")
                    , train.settings.get<Acceleration>("acceleration_travel_layer_0")
                    , train.settings.get<Velocity>("jerk_travel_layer_0")
            };
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

            travel.smoothSpeed(initial_layer_travel_speed_config, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);

            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            const GCodePathConfig::SpeedDerivatives& initial_layer_print_speed_config = global_first_layer_config_per_extruder[extruder_nr];

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
            prime_tower.smoothSpeed(initial_layer_print_speed_config, std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
        }

    }

    { // meshes
        for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = storage.meshes[mesh_idx];

            GCodePathConfig::SpeedDerivatives initial_layer_speed_config{
                    mesh.settings.get<Velocity>("speed_print_layer_0")
                    , mesh.settings.get<Acceleration>("acceleration_print_layer_0")
                    , mesh.settings.get<Velocity>("jerk_print_layer_0")
            };

            mesh_configs[mesh_idx].smoothAllSpeeds(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
            mesh_configs[mesh_idx].roofing_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
        }
    }
}

}//namespace cura
