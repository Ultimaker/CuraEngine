/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "PathConfigStorage.h"

#include "settings.h" // MAX_INFILL_COMBINE
#include "../sliceDataStorage.h" // SliceDataStorage


namespace cura
{

GCodePathConfig getPerimeterGapConfig(const SliceMeshStorage& mesh, int layer_thickness)
{
    // The perimeter gap config follows the skin config, but has a different line width:
    // wall_line_width_x divided by two because the gaps are between 0 and 1 times the wall line width
    const int perimeter_gaps_line_width = mesh.getSettingInMicrons("wall_line_width_x") / 2;
    double perimeter_gaps_speed = mesh.getSettingInMillimetersPerSecond("speed_topbottom");
    if (mesh.getSettingBoolean("speed_equalize_flow_enabled"))
    {
        perimeter_gaps_speed = perimeter_gaps_speed * mesh.getSettingInMicrons("skin_line_width") / perimeter_gaps_line_width;
    }
    return GCodePathConfig(
            PrintFeatureType::Skin
            , perimeter_gaps_line_width
            , layer_thickness
            , mesh.getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{perimeter_gaps_speed, mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom")}
        );
}

PathConfigStorage::MeshPathConfigs::MeshPathConfigs(const SliceMeshStorage& mesh, int layer_thickness)
: inset0_config(
    PrintFeatureType::OuterWall
    , mesh.getSettingInMicrons("wall_line_width_0")
    , layer_thickness
    , mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0")}
)
, insetX_config(
    PrintFeatureType::InnerWall
    , mesh.getSettingInMicrons("wall_line_width_x")
    , layer_thickness
    , mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x")}
)
, skin_config(
    PrintFeatureType::Skin
    , mesh.getSettingInMicrons("skin_line_width")
    , layer_thickness
    , mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_topbottom"), mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom")}
)

, perimeter_gap_config(getPerimeterGapConfig(mesh, layer_thickness))
{
    infill_config.reserve(MAX_INFILL_COMBINE);
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        infill_config.emplace_back(
                PrintFeatureType::Infill
                , mesh.getSettingInMicrons("infill_line_width") * (combine_idx + 1)
                , layer_thickness
                , mesh.getSettingInPercentage("material_flow")
                , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_infill"), mesh.getSettingInMillimetersPerSecond("jerk_infill")}
            );
    }
}

PathConfigStorage::PathConfigStorage(const SliceDataStorage& storage, int layer_nr, int layer_thickness)
: adhesion_extruder_train(storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr")))
, support_infill_train(storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr")))
, support_interface_train(storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_interface_extruder_nr")))
, raft_base_config(
            PrintFeatureType::SupportInterface
            , adhesion_extruder_train->getSettingInMicrons("raft_base_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_base_thickness")
            , adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_jerk")}
        )
, raft_interface_config(
            PrintFeatureType::Support
            , adhesion_extruder_train->getSettingInMicrons("raft_interface_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_interface_thickness")
            , adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_jerk")}
        )
, raft_surface_config(
            PrintFeatureType::SupportInterface
            , adhesion_extruder_train->getSettingInMicrons("raft_surface_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_surface_thickness")
            , adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_jerk")}
        )
, support_infill_config(
            PrintFeatureType::Support
            , support_infill_train->getSettingInMicrons("support_line_width")
            , layer_thickness
            , support_infill_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_infill_train->getSettingInMillimetersPerSecond("speed_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("acceleration_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("jerk_support_infill")}
        )
, support_roof_config(
            PrintFeatureType::SupportInterface
            , support_interface_train->getSettingInMicrons("support_interface_line_width")
            , layer_thickness
            , support_interface_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_interface_train->getSettingInMillimetersPerSecond("speed_support_interface"), support_interface_train->getSettingInMillimetersPerSecond("acceleration_support_interface"), support_interface_train->getSettingInMillimetersPerSecond("jerk_support_interface")}
        )
, support_bottom_config(
            PrintFeatureType::SupportInterface
            , support_interface_train->getSettingInMicrons("support_interface_line_width")
            , layer_thickness
            , support_interface_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_interface_train->getSettingInMillimetersPerSecond("speed_support_interface"), support_interface_train->getSettingInMillimetersPerSecond("acceleration_support_interface"), support_interface_train->getSettingInMillimetersPerSecond("jerk_support_interface")}
        )
{
    const int extruder_count = storage.meshgroup->getExtruderCount();
    travel_config_per_extruder.reserve(extruder_count);
    skirt_brim_config_per_extruder.reserve(extruder_count);
    prime_tower_config_per_extruder.reserve(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
        travel_config_per_extruder.emplace_back(
                PrintFeatureType::MoveCombing
                , 0
                , 0
                , 0.0
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("speed_travel"), train->getSettingInMillimetersPerSecond("acceleration_travel"), train->getSettingInMillimetersPerSecond("jerk_travel")}
            );
        skirt_brim_config_per_extruder.emplace_back(
                PrintFeatureType::SkirtBrim
                , train->getSettingInMicrons("skirt_brim_line_width")
                , layer_thickness
                , train->getSettingInPercentage("material_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("skirt_brim_speed"), train->getSettingInMillimetersPerSecond("acceleration_skirt_brim"), train->getSettingInMillimetersPerSecond("jerk_skirt_brim")}
            );
        prime_tower_config_per_extruder.emplace_back(
                PrintFeatureType::SupportInfill
                , train->getSettingInMicrons("prime_tower_line_width")
                , layer_thickness
                , train->getSettingInPercentage("prime_tower_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower")}
            );
    }

    mesh_configs.reserve(storage.meshes.size());
    for (const SliceMeshStorage& mesh_storage : storage.meshes)
    {
        mesh_configs.emplace_back(mesh_storage, layer_thickness);
    }

    const int initial_speedup_layer_count = storage.getSettingAsCount("speed_slowdown_layers");
    if (layer_nr < initial_speedup_layer_count)
    {
        handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
    }
}

void cura::PathConfigStorage::handleInitialLayerSpeedup(const SliceDataStorage& storage, int layer_nr, int initial_speedup_layer_count)
{
    std::vector<GCodePathConfig::SpeedDerivatives> global_first_layer_config_per_extruder;
    global_first_layer_config_per_extruder.reserve(storage.meshgroup->getExtruderCount());
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        const ExtruderTrain* extruder = storage.meshgroup->getExtruderTrain(extruder_nr);
        global_first_layer_config_per_extruder.emplace_back(
            GCodePathConfig::SpeedDerivatives{
                extruder->getSettingInMillimetersPerSecond("speed_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("jerk_print_layer_0")
            });
    }

    { // support
        if (layer_nr < initial_speedup_layer_count)
        {
            const int extruder_nr_support_infill = storage.getSettingAsIndex((layer_nr <= 0)? "support_extruder_nr_layer_0" : "support_infill_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_infill = global_first_layer_config_per_extruder[extruder_nr_support_infill];
            support_infill_config.smoothSpeed(first_layer_config_infill, std::max(0, layer_nr), initial_speedup_layer_count);

            const int extruder_nr_support_interface = storage.getSettingAsIndex("support_interface_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_interface = global_first_layer_config_per_extruder[extruder_nr_support_interface];
            support_roof_config.smoothSpeed(first_layer_config_interface, std::max(0, layer_nr), initial_speedup_layer_count);
            support_bottom_config.smoothSpeed(first_layer_config_interface, std::max(0, layer_nr), initial_speedup_layer_count);
        }
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            GCodePathConfig::SpeedDerivatives initial_layer_speed_config{
                    train->getSettingInMillimetersPerSecond("speed_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("acceleration_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("jerk_travel_layer_0")
            };
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

            travel.smoothSpeed(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);

            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];

            prime_tower.smoothSpeed(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
        }

    }

    { // meshes
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = storage.meshes[mesh_idx];


            GCodePathConfig::SpeedDerivatives initial_layer_speed_config{
                    mesh.getSettingInMillimetersPerSecond("speed_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("jerk_print_layer_0")
            };

            MeshPathConfigs& mesh_config = mesh_configs[mesh_idx];
            //Outer wall speed (per mesh).
            mesh_config.inset0_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);

            //Inner wall speed (per mesh).
            mesh_config.insetX_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);

            //Skin speed (per mesh).
            mesh_config.skin_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
            mesh_config.perimeter_gap_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);

            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                //Infill speed (per combine part per mesh).
                mesh_config.infill_config[idx].smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
            }
        }
    }
}


}//namespace cura
