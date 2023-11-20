// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SETTINGS_TYPES_MESHPATHCONFIGS_H
#define SETTINGS_TYPES_MESHPATHCONFIGS_H

#include "GCodePathConfig.h"
#include "settings/types/LayerIndex.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"

namespace cura
{
struct MeshPathConfigs
{
    GCodePathConfig inset0_config{};
    GCodePathConfig insetX_config{};
    GCodePathConfig inset0_roofing_config{};
    GCodePathConfig insetX_roofing_config{};
    GCodePathConfig bridge_inset0_config{};
    GCodePathConfig bridge_insetX_config{};
    GCodePathConfig skin_config{};
    GCodePathConfig bridge_skin_config{}; // used for first bridge layer
    GCodePathConfig bridge_skin_config2{}; // used for second bridge layer
    GCodePathConfig bridge_skin_config3{}; // used for third bridge layer
    GCodePathConfig roofing_config{};
    std::vector<GCodePathConfig> infill_config{};
    GCodePathConfig ironing_config{};

    MeshPathConfigs(const SliceMeshStorage& mesh, const coord_t layer_thickness, const LayerIndex layer_nr, const std::vector<Ratio>& line_width_factor_per_extruder);
    void smoothAllSpeeds(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer);
};

} // namespace cura

#endif // SETTINGS_TYPES_MESHPATHCONFIGS_H
