// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SETTINGS_PATH_CONFIGS_H
#define SETTINGS_PATH_CONFIGS_H

#include <vector>

#include "GCodePathConfig.h"
#include "pathPlanning/SpeedDerivatives.h"
#include "settings/MeshPathConfigs.h"
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"

namespace cura
{

class ExtruderTrain;
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
    ExtruderTrain& raft_base_train;
    ExtruderTrain& raft_interface_train;
    ExtruderTrain& raft_surface_train;
    ExtruderTrain& support_infill_train;
    ExtruderTrain& support_roof_train;
    ExtruderTrain& support_bottom_train;

    const std::vector<Ratio> line_width_factor_per_extruder;
    static std::vector<Ratio> getLineWidthFactorPerExtruder(const LayerIndex& layer_nr);

public:
    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)
    std::vector<GCodePathConfig> skirt_brim_config_per_extruder; //!< Configuration for skirt and brim per extruder.
    std::vector<GCodePathConfig> prime_tower_config_per_extruder; //!< Configuration for the prime tower per extruder.

    std::vector<GCodePathConfig> support_infill_config; //!< The config used to print the normal support, rather than the support interface
    std::vector<GCodePathConfig> support_fractional_infill_config; //!< The config used to print the normal support on fractional layer-height parts.
    GCodePathConfig support_roof_config; //!< The config used to print the dense roofs of support.
    GCodePathConfig support_fractional_roof_config; //!< The config used to print the dense roofs of support on fractional layer-height parts.
    GCodePathConfig support_bottom_config; //!< The config to use to print the dense bottoms of support

    std::vector<MeshPathConfigs> mesh_configs; //!< For each meash the config for all its feature types

    /*!
     * \warning Note that the layer_nr might be below zero for raft (filler) layers
     */
    PathConfigStorage(const SliceDataStorage& storage, const LayerIndex& layer_nr, const coord_t layer_thickness);

private:
    void handleInitialLayerSpeedup(const SliceDataStorage& storage, const LayerIndex& layer_nr, const size_t initial_speedup_layer_count);
};

} // namespace cura

#endif // SETTINGS_PATH_CONFIGS_H
