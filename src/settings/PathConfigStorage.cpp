/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "PathConfigStorage.h"

#include "settings.h" // MAX_INFILL_COMBINE
#include "../sliceDataStorage.h" // SliceDataStorage
#include "../raft.h"

namespace cura
{

std::vector<double> PathConfigStorage::getLineWidthFactorPerExtruder(const SliceDataStorage& storage, int layer_nr)
{
    std::vector<double> ret;
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        if (layer_nr <= 0)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            double factor = train->getSettingAsRatio("initial_layer_line_width_factor");
            ret.push_back(factor);
        }
        else
        {
            ret.push_back(1.0);
        }
    }
    return ret;
}

GCodePathConfig createPerimeterGapConfig(const SliceMeshStorage& mesh, int layer_thickness, int layer_nr)
{
    // The perimeter gap config follows the skin config, but has a different line width:
    // wall_line_width_x divided by two because the gaps are between 0 and 1 times the wall line width
    int perimeter_gaps_line_width = mesh.getSettingInMicrons("wall_line_width_0") / 2;
    double perimeter_gaps_speed = mesh.getSettingInMillimetersPerSecond("speed_topbottom");
    if (mesh.getSettingBoolean("speed_equalize_flow_enabled"))
    {
        int skin_line_width = mesh.getSettingInMicrons("skin_line_width");
        perimeter_gaps_speed *= skin_line_width / perimeter_gaps_line_width;
    }
    return GCodePathConfig(
            PrintFeatureType::Skin
            , perimeter_gaps_line_width
            , layer_thickness
            , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{perimeter_gaps_speed, mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom")}
        );
}

PathConfigStorage::MeshPathConfigs::MeshPathConfigs(const SliceMeshStorage& mesh, int layer_thickness, int layer_nr, const std::vector<double>& line_width_factor_per_extruder)
: inset0_config(
    PrintFeatureType::OuterWall
    , mesh.getSettingInMicrons("wall_line_width_0") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_0_extruder_nr")]
    , layer_thickness
    , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0")}
)
, insetX_config(
    PrintFeatureType::InnerWall
    , mesh.getSettingInMicrons("wall_line_width_x") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_x_extruder_nr")]
    , layer_thickness
    , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x")}
)
, skin_config(
    PrintFeatureType::Skin
    , mesh.getSettingInMicrons("skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
    , layer_thickness
    , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_topbottom"), mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom")}
)
, roofing_config(
    PrintFeatureType::Skin
    , mesh.getSettingInMicrons("roofing_line_width")
    , layer_thickness
    , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_roofing"), mesh.getSettingInMillimetersPerSecond("acceleration_roofing"), mesh.getSettingInMillimetersPerSecond("jerk_roofing")}
)
, ironing_config(
    PrintFeatureType::Skin
    , mesh.getSettingInMicrons("skin_line_width")
    , layer_thickness
    , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_ironing"), mesh.getSettingInMillimetersPerSecond("acceleration_ironing"), mesh.getSettingInMillimetersPerSecond("jerk_ironing")}
)

, perimeter_gap_config(createPerimeterGapConfig(mesh, layer_thickness, layer_nr))
{
    infill_config.reserve(MAX_INFILL_COMBINE);
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        infill_config.emplace_back(
                PrintFeatureType::Infill
                , mesh.getSettingInMicrons("infill_line_width") * (combine_idx + 1) * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("infill_extruder_nr")]
                , layer_thickness
                , (layer_nr == 0)? mesh.getSettingInPercentage("material_flow_layer_0") : mesh.getSettingInPercentage("material_flow")
                , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_infill"), mesh.getSettingInMillimetersPerSecond("jerk_infill")}
            );
    }
}

PathConfigStorage::PathConfigStorage(const SliceDataStorage& storage, int layer_nr, int layer_thickness)
: adhesion_extruder_nr(storage.getSettingAsIndex("adhesion_extruder_nr"))
, support_infill_extruder_nr(storage.getSettingAsIndex("support_infill_extruder_nr"))
, support_roof_extruder_nr(storage.getSettingAsIndex("support_roof_extruder_nr"))
, support_bottom_extruder_nr(storage.getSettingAsIndex("support_bottom_extruder_nr"))
, adhesion_extruder_train(storage.meshgroup->getExtruderTrain(adhesion_extruder_nr))
, support_infill_train(storage.meshgroup->getExtruderTrain(support_infill_extruder_nr))
, support_roof_train(storage.meshgroup->getExtruderTrain(support_roof_extruder_nr))
, support_bottom_train(storage.meshgroup->getExtruderTrain(support_bottom_extruder_nr))
, line_width_factor_per_extruder(PathConfigStorage::getLineWidthFactorPerExtruder(storage, layer_nr))
, raft_base_config(
            PrintFeatureType::SupportInterface
            , adhesion_extruder_train->getSettingInMicrons("raft_base_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_base_thickness")
            , (layer_nr == 0)? adhesion_extruder_train->getSettingInPercentage("material_flow_layer_0") : adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_base_jerk")}
        )
, raft_interface_config(
            PrintFeatureType::Support
            , adhesion_extruder_train->getSettingInMicrons("raft_interface_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_interface_thickness")
            , (layer_nr == 0)? adhesion_extruder_train->getSettingInPercentage("material_flow_layer_0") : adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_interface_jerk")}
        )
, raft_surface_config(
            PrintFeatureType::SupportInterface
            , adhesion_extruder_train->getSettingInMicrons("raft_surface_line_width")
            , adhesion_extruder_train->getSettingInMicrons("raft_surface_thickness")
            , (layer_nr == 0)? adhesion_extruder_train->getSettingInPercentage("material_flow_layer_0") : adhesion_extruder_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_speed"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_acceleration"), adhesion_extruder_train->getSettingInMillimetersPerSecond("raft_surface_jerk")}
        )
, support_roof_config(
            PrintFeatureType::SupportInterface
            , support_roof_train->getSettingInMicrons("support_roof_line_width") * line_width_factor_per_extruder[support_roof_extruder_nr]
            , layer_thickness
            , (layer_nr == 0)? support_roof_train->getSettingInPercentage("material_flow_layer_0") : support_roof_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_roof_train->getSettingInMillimetersPerSecond("speed_support_roof"), support_roof_train->getSettingInMillimetersPerSecond("acceleration_support_roof"), support_roof_train->getSettingInMillimetersPerSecond("jerk_support_roof")}
        )
, support_bottom_config(
            PrintFeatureType::SupportInterface
            , support_bottom_train->getSettingInMicrons("support_bottom_line_width") * line_width_factor_per_extruder[support_bottom_extruder_nr]
            , layer_thickness
            , (layer_nr == 0)? support_bottom_train->getSettingInPercentage("material_flow_layer_0") : support_bottom_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_bottom_train->getSettingInMillimetersPerSecond("speed_support_bottom"), support_bottom_train->getSettingInMillimetersPerSecond("acceleration_support_bottom"), support_bottom_train->getSettingInMillimetersPerSecond("jerk_support_bottom")}
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
                    * ((storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[extruder_nr]) // cause it's also used for the draft/ooze shield
                , layer_thickness
                , (layer_nr == 0)? train->getSettingInPercentage("material_flow_layer_0") : train->getSettingInPercentage("material_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("skirt_brim_speed"), train->getSettingInMillimetersPerSecond("acceleration_skirt_brim"), train->getSettingInMillimetersPerSecond("jerk_skirt_brim")}
            );
        prime_tower_config_per_extruder.emplace_back(
                PrintFeatureType::SupportInfill
                , train->getSettingInMicrons("prime_tower_line_width")
                    * ((storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[extruder_nr])
                , layer_thickness
                , train->getSettingInPercentage("prime_tower_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower")}
            );
    }

    mesh_configs.reserve(storage.meshes.size());
    for (const SliceMeshStorage& mesh_storage : storage.meshes)
    {
        mesh_configs.emplace_back(mesh_storage, layer_thickness, layer_nr, line_width_factor_per_extruder);
    }

    support_infill_config.reserve(MAX_INFILL_COMBINE);
    const float support_infill_line_width_factor = (storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[support_infill_extruder_nr];
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        support_infill_config.emplace_back(
            PrintFeatureType::Support
            , support_infill_train->getSettingInMicrons("support_line_width") * (combine_idx + 1) * support_infill_line_width_factor
            , layer_thickness
            , (layer_nr == 0)? support_infill_train->getSettingInPercentage("material_flow_layer_0") : support_infill_train->getSettingInPercentage("material_flow")
            , GCodePathConfig::SpeedDerivatives{support_infill_train->getSettingInMillimetersPerSecond("speed_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("acceleration_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("jerk_support_infill")}
        );
    }

    const int initial_speedup_layer_count = storage.getSettingAsCount("speed_slowdown_layers");
    if (layer_nr < initial_speedup_layer_count)
    {
        handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
    }
}

void PathConfigStorage::MeshPathConfigs::smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer)
{
    inset0_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
    insetX_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
    skin_config.smoothSpeed(                first_layer_config, layer_nr, max_speed_layer);
    ironing_config.smoothSpeed(             first_layer_config, layer_nr, max_speed_layer);
    perimeter_gap_config.smoothSpeed(       first_layer_config, layer_nr, max_speed_layer);
    for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
    {
        //Infill speed (per combine part per mesh).
        infill_config[idx].smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
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
            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                support_infill_config[idx].smoothSpeed(first_layer_config_infill, std::max(0, layer_nr), initial_speedup_layer_count);
            }

            const int extruder_nr_support_roof = storage.getSettingAsIndex("support_roof_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_roof = global_first_layer_config_per_extruder[extruder_nr_support_roof];
            support_roof_config.smoothSpeed(first_layer_config_roof, std::max(0, layer_nr), initial_speedup_layer_count);
            const int extruder_nr_support_bottom = storage.getSettingAsIndex("support_bottom_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_bottom = global_first_layer_config_per_extruder[extruder_nr_support_bottom];
            support_bottom_config.smoothSpeed(first_layer_config_bottom, std::max(0, layer_nr), initial_speedup_layer_count);
        }
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            GCodePathConfig::SpeedDerivatives initial_layer_travel_speed_config{
                    train->getSettingInMillimetersPerSecond("speed_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("acceleration_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("jerk_travel_layer_0")
            };
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

            travel.smoothSpeed(initial_layer_travel_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);

            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            const GCodePathConfig::SpeedDerivatives& initial_layer_print_speed_config = global_first_layer_config_per_extruder[extruder_nr];

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
            prime_tower.smoothSpeed(initial_layer_print_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
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

            mesh_configs[mesh_idx].smoothAllSpeeds(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
            mesh_configs[mesh_idx].roofing_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
        }
    }
}


}//namespace cura
