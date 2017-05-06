/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_PATH_CONFIGS_H
#define SETTINGS_PATH_CONFIGS_H

#include <vector>

#include "../utils/intpoint.h" // coord_t
#include "../GCodePathConfig.h"

namespace cura
{

class SliceDataStorage; // forward decl for SliceDataStorage
class SliceMeshStorage; // forward decl for SliceDataStorage
class ExtruderTrain; // forward decl for SliceDataStorage

/*!
 * A class to represent all configurations for all features types of printed lines in a meshgroup.
 */
class PathConfigStorage
{
private:
    const ExtruderTrain* adhesion_extruder_train;
    const ExtruderTrain* support_infill_train;
    const ExtruderTrain* support_roof_train;
    const ExtruderTrain* support_bottom_train;
public:
    class MeshPathConfigs
    {
    public:
        GCodePathConfig inset0_config;
        GCodePathConfig inset0_config_layer0;
        GCodePathConfig insetX_config;
        GCodePathConfig insetX_config_layer0;
        GCodePathConfig skin_config;
        GCodePathConfig skin_config_layer0;
        GCodePathConfig perimeter_gap_config;
        std::vector<GCodePathConfig> infill_config;

        MeshPathConfigs(const SliceMeshStorage& mesh, int layer_thickness);
        const GCodePathConfig *getInset0Config(const int layer_nr) const;
        const GCodePathConfig *getInsetXConfig(const int layer_nr) const;
        const GCodePathConfig *getSkinConfig(const int layer_nr) const;
    };
    
    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)
    std::vector<GCodePathConfig> skirt_brim_config_per_extruder; //!< Configuration for skirt and brim per extruder.
    std::vector<GCodePathConfig> prime_tower_config_per_extruder; //!< Configuration for the prime tower per extruder.

    GCodePathConfig support_infill_config; //!< The config used to print the normal support, rather than the support interface
    GCodePathConfig support_roof_config; //!< The config used to print the dense roofs of support.
    GCodePathConfig support_bottom_config; //!< The config to use to print the dense bottoms of support

    std::vector<MeshPathConfigs> mesh_configs; //!< For each meash the config for all its feature types
    
    /*!
     * \warning Note that the layer_nr might be below zero for raft (filler) layers
     */
    PathConfigStorage(const SliceDataStorage& storage, int layer_nr, int layer_thickness);

private:
    void handleInitialLayerSpeedup(const SliceDataStorage& storage, int layer_nr, int initial_speedup_layer_count);
};

}; // namespace cura

#endif // SETTINGS_PATH_CONFIGS_H