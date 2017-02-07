/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_PATH_CONFIGS_H
#define SETTINGS_PATH_CONFIGS_H

#include <vector>

#include "../utils/intpoint.h" // coord_t
#include "../GCodePathConfig.h"

namespace cura
{

class SliceDataStorage; // forward decl for SliceDataStorage
/*!
 * A class to represent all configurations for all features types of printed lines in a meshgroup.
 */
class PathConfigs
{
public:
    class MeshPathConfigs
    {
    public:
        GCodePathConfig inset0_config;
        GCodePathConfig insetX_config;
        GCodePathConfig skin_config;
        GCodePathConfig perimeter_gap_config;
        std::vector<GCodePathConfig> infill_config;

        MeshPathConfigs();
    };
    
    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)
    std::vector<GCodePathConfig> skirt_brim_config; //!< Configuration for skirt and brim per extruder.
    std::vector<GCodePathConfig> prime_tower_config_per_extruder; //!< Configuration for the prime tower per extruder.

    GCodePathConfig support_infill_config; //!< The config used to print the normal support, rather than the support interface
    GCodePathConfig support_interface_config; //!< The config to use to print the dense roofs and bottoms of support

    std::vector<MeshPathConfigs> mesh_configs; //!< For each meash the config for all its feature types
    
    /*!
     * \warning Note that the layer_nr might be below zero for raft (filler) layers
     */
    PathConfigs(const SliceDataStorage& storage, int layer_nr, coord_t layer_thickness);
};

}; // namespace cura

#endif // SETTINGS_PATH_CONFIGS_H