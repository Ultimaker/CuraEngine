//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_PATH_CONFIGS_H
#define SETTINGS_PATH_CONFIGS_H

#include <vector>

#include "../GCodePathConfig.h"
#include "../utils/Coord_t.h"

namespace cura
{

class ExtruderTrain;
struct LayerIndex;
class SliceDataStorage;
class SliceMeshStorage;

/*!
 * A class to represent all configurations for all features types of printed lines in a meshgroup.
 */
class PathConfigStorage
{
private:
    const size_t support_infill_extruder_nr;
    const size_t support_roof_extruder_nr;
    const size_t support_bottom_extruder_nr;
    const ExtruderTrain& adhesion_extruder_train;
    const ExtruderTrain& support_infill_train;
    const ExtruderTrain& support_roof_train;
    const ExtruderTrain& support_bottom_train;

    const std::vector<Ratio> line_width_factor_per_extruder;
    static std::vector<Ratio> getLineWidthFactorPerExtruder(const LayerIndex& layer_nr);
public:
    class MeshPathConfigs
    {
    public:
        GCodePathConfig inset0_config;
        GCodePathConfig insetX_config;
        GCodePathConfig bridge_inset0_config;
        GCodePathConfig bridge_insetX_config;
        GCodePathConfig skin_config;
        GCodePathConfig bridge_skin_config;  // used for first bridge layer
        GCodePathConfig bridge_skin_config2; // used for second bridge layer
        GCodePathConfig bridge_skin_config3; // used for third bridge layer
        GCodePathConfig roofing_config;
        std::vector<GCodePathConfig> infill_config;
        GCodePathConfig ironing_config;
        GCodePathConfig perimeter_gap_config;

        MeshPathConfigs(const SliceMeshStorage& mesh, const coord_t layer_thickness, const LayerIndex& layer_nr, const std::vector<Ratio>& line_width_factor_per_extruder);
        void smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, const LayerIndex& layer_nr, const LayerIndex& max_speed_layer);
    };

    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)
    std::vector<GCodePathConfig> skirt_brim_config_per_extruder; //!< Configuration for skirt and brim per extruder.
    std::vector<GCodePathConfig> prime_tower_config_per_extruder; //!< Configuration for the prime tower per extruder.

    std::vector<GCodePathConfig> support_infill_config; //!< The config used to print the normal support, rather than the support interface
    GCodePathConfig support_roof_config; //!< The config used to print the dense roofs of support.
    GCodePathConfig support_bottom_config; //!< The config to use to print the dense bottoms of support

    std::vector<MeshPathConfigs> mesh_configs; //!< For each meash the config for all its feature types

    /*!
     * \warning Note that the layer_nr might be below zero for raft (filler) layers
     */
    PathConfigStorage(const SliceDataStorage& storage, const LayerIndex& layer_nr, const coord_t layer_thickness);

private:
    void handleInitialLayerSpeedup(const SliceDataStorage& storage, const LayerIndex& layer_nr, const size_t initial_speedup_layer_count);
};

}; // namespace cura

#endif // SETTINGS_PATH_CONFIGS_H
