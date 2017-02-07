/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "PathConfigs.h"

#include "settings.h" // MAX_INFILL_COMBINE
#include "../sliceDataStorage.h" // SliceDataStorage


namespace cura
{

PathConfigs::MeshPathConfigs::MeshPathConfigs()
: inset0_config(PrintFeatureType::OuterWall)
, insetX_config(PrintFeatureType::InnerWall)
, skin_config(PrintFeatureType::Skin)
, perimeter_gap_config(PrintFeatureType::Skin)
{
    infill_config.reserve(MAX_INFILL_COMBINE);
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        infill_config.emplace_back(PrintFeatureType::Infill);
    }
}

PathConfigs::PathConfigs(const SliceDataStorage& storage, int layer_nr, coord_t layer_thickness)
: raft_base_config(PrintFeatureType::SupportInterface)
, raft_interface_config(PrintFeatureType::Support)
, raft_surface_config(PrintFeatureType::SupportInterface)
, support_infill_config(PrintFeatureType::Support)
, support_interface_config(PrintFeatureType::SupportInterface)
{
    int extruder_count = storage.meshgroup->getExtruderCount();
    travel_config_per_extruder.reserve(extruder_count);
    skirt_brim_config.reserve(extruder_count);
    prime_tower_config_per_extruder.reserve(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        travel_config_per_extruder.emplace_back(PrintFeatureType::MoveCombing);
        skirt_brim_config.emplace_back(PrintFeatureType::SkirtBrim);
        prime_tower_config_per_extruder.emplace_back(PrintFeatureType::SupportInfill);
    }

    mesh_configs.resize(storage.meshes.size());
    
    
    int initial_speedup_layer_count = storage.getSettingAsCount("speed_slowdown_layers");

    constexpr int iconic_line_width_unused = std::numeric_limits<int>::lowest();
    constexpr double iconic_flow_unused = std::numeric_limits<double>::infinity();

    std::vector<GCodePathConfig::BasicConfig> global_first_layer_config_per_extruder;
    global_first_layer_config_per_extruder.reserve(storage.meshgroup->getExtruderCount());
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        const ExtruderTrain* extruder = storage.meshgroup->getExtruderTrain(extruder_nr);
        global_first_layer_config_per_extruder.emplace_back(
                extruder->getSettingInMillimetersPerSecond("speed_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("jerk_print_layer_0")
                , iconic_line_width_unused
                , iconic_flow_unused
            );
    }

    { // support
        SettingsBase* infill_train = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
        support_infill_config.setLayerHeight(layer_thickness);
        support_infill_config.init(infill_train->getSettingInMillimetersPerSecond("speed_support_infill"), infill_train->getSettingInMillimetersPerSecond("acceleration_support_infill"), infill_train->getSettingInMillimetersPerSecond("jerk_support_infill"), infill_train->getSettingInMicrons("support_line_width"), infill_train->getSettingInPercentage("material_flow"));

        SettingsBase* interface_train = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_interface_extruder_nr"));
        support_interface_config.setLayerHeight(layer_thickness);
        support_interface_config.init(interface_train->getSettingInMillimetersPerSecond("speed_support_interface"), interface_train->getSettingInMillimetersPerSecond("acceleration_support_interface"), interface_train->getSettingInMillimetersPerSecond("jerk_support_interface"), interface_train->getSettingInMicrons("support_interface_line_width"), interface_train->getSettingInPercentage("material_flow"));
        if (layer_nr < initial_speedup_layer_count)
        {
            int extruder_nr_support_infill = storage.getSettingAsIndex((layer_nr <= 0)? "support_extruder_nr_layer_0" : "support_infill_extruder_nr");
            GCodePathConfig::BasicConfig& first_layer_config_infill = global_first_layer_config_per_extruder[extruder_nr_support_infill];
            support_infill_config.smoothSpeed(first_layer_config_infill, std::max(0, layer_nr), initial_speedup_layer_count);

            int extruder_nr_support_interface = storage.getSettingAsIndex("support_interface_extruder_nr");
            GCodePathConfig::BasicConfig& first_layer_config_interface = global_first_layer_config_per_extruder[extruder_nr_support_interface];
            support_interface_config.smoothSpeed(first_layer_config_interface, std::max(0, layer_nr), initial_speedup_layer_count);
        }
        else
        { // TODO: remove setSpeedIconic
            support_infill_config.setSpeedIconic();
            support_interface_config.setSpeedIconic();
        }
    }

    { // other helper parts: prime_tower, raft
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"));
        raft_base_config.init(train->getSettingInMillimetersPerSecond("raft_base_speed"), train->getSettingInMillimetersPerSecond("raft_base_acceleration"), train->getSettingInMillimetersPerSecond("raft_base_jerk"), train->getSettingInMicrons("raft_base_line_width"), train->getSettingInPercentage("material_flow"));
        raft_base_config.setLayerHeight(train->getSettingInMicrons("raft_base_thickness"));
        
        raft_interface_config.init(train->getSettingInMillimetersPerSecond("raft_interface_speed"), train->getSettingInMillimetersPerSecond("raft_interface_acceleration"), train->getSettingInMillimetersPerSecond("raft_interface_jerk"), train->getSettingInMicrons("raft_interface_line_width"), train->getSettingInPercentage("material_flow"));
        raft_interface_config.setLayerHeight(train->getSettingInMicrons("raft_interface_thickness"));

        raft_surface_config.init(train->getSettingInMillimetersPerSecond("raft_surface_speed"), train->getSettingInMillimetersPerSecond("raft_surface_acceleration"), train->getSettingInMillimetersPerSecond("raft_surface_jerk"), train->getSettingInMicrons("raft_surface_line_width"), train->getSettingInPercentage("material_flow"));
        raft_surface_config.setLayerHeight(train->getSettingInMicrons("raft_surface_thickness"));
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            GCodePathConfig::BasicConfig initial_layer_speed_config(
                    train->getSettingInMillimetersPerSecond("speed_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("acceleration_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("jerk_travel_layer_0")
                    , iconic_line_width_unused
                    , iconic_flow_unused
                );
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];
            travel.init(train->getSettingInMillimetersPerSecond("speed_travel"), train->getSettingInMillimetersPerSecond("acceleration_travel"), train->getSettingInMillimetersPerSecond("jerk_travel"), 0, 0);

            travel.setLayerHeight(layer_thickness);
            travel.smoothSpeed(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);

            GCodePathConfig& skirt_brim = skirt_brim_config[extruder_nr];
            skirt_brim.init(train->getSettingInMillimetersPerSecond("skirt_brim_speed"), train->getSettingInMillimetersPerSecond("acceleration_skirt_brim"), train->getSettingInMillimetersPerSecond("jerk_skirt_brim"), train->getSettingInMicrons("skirt_brim_line_width"), train->getSettingInPercentage("material_flow"));
            skirt_brim.setLayerHeight(layer_thickness);
            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
            prime_tower.init(train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower"), train->getSettingInMicrons("prime_tower_line_width"), train->getSettingInPercentage("prime_tower_flow"));

            prime_tower.setLayerHeight(layer_thickness);
            prime_tower.smoothSpeed(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
        }

    }

    { // meshes
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = storage.meshes[mesh_idx];

            // The perimeter gap config follows the skin config, but has a different line width:
            // wall_line_width_x divided by two because the gaps are between 0 and 1 times the wall line width
            const int perimeter_gaps_line_width = mesh.getSettingInMicrons("wall_line_width_x") / 2;
            double perimeter_gaps_speed = mesh.getSettingInMillimetersPerSecond("speed_topbottom");
            if (mesh.getSettingBoolean("speed_equalize_flow_enabled"))
            {
                perimeter_gaps_speed = perimeter_gaps_speed * mesh.getSettingInMicrons("skin_line_width") / perimeter_gaps_line_width;
            }

            GCodePathConfig::BasicConfig initial_layer_speed_config(
                    mesh.getSettingInMillimetersPerSecond("speed_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("jerk_print_layer_0")
                    , iconic_line_width_unused
                    , iconic_flow_unused
                );

            MeshPathConfigs& mesh_config = mesh_configs[mesh_idx];
            mesh_config.inset0_config.setLayerHeight(layer_thickness);
            mesh_config.inset0_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0"), mesh.getSettingInMicrons("wall_line_width_0"), mesh.getSettingInPercentage("material_flow"));
            mesh_config.insetX_config.setLayerHeight(layer_thickness);
            mesh_config.insetX_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x"), mesh.getSettingInMicrons("wall_line_width_x"), mesh.getSettingInPercentage("material_flow"));
            mesh_config.skin_config.setLayerHeight(layer_thickness);
            mesh_config.skin_config.init(mesh.getSettingInMillimetersPerSecond("speed_topbottom"), mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom"), mesh.getSettingInMicrons("skin_line_width"), mesh.getSettingInPercentage("material_flow"));
            mesh_config.perimeter_gap_config.setLayerHeight(layer_thickness);
            mesh_config.perimeter_gap_config.init(perimeter_gaps_speed, mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom"), perimeter_gaps_line_width, mesh.getSettingInPercentage("material_flow"));
            for (unsigned int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
            {
                mesh_config.infill_config[combine_idx].setLayerHeight(layer_thickness);
                mesh_config.infill_config[combine_idx].init(mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_infill"), mesh.getSettingInMillimetersPerSecond("jerk_infill"), mesh.getSettingInMicrons("infill_line_width") * (combine_idx + 1), mesh.getSettingInPercentage("material_flow"));
            }
            
            if (layer_nr < initial_speedup_layer_count)
            {
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
            else
            { // TODO: remove setSpeedIconic
                mesh_config.inset0_config.setSpeedIconic();
                mesh_config.insetX_config.setSpeedIconic();
                mesh_config.skin_config.setSpeedIconic();
                mesh_config.perimeter_gap_config.setSpeedIconic();
                for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
                {
                    //Infill speed (per combine part per mesh).
                    mesh_config.infill_config[idx].setSpeedIconic();
                }
            }
        }
    }
}


}//namespace cura
