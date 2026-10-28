// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>
#include <vector>

#include "SupportLayer.h"
#include "types/AngleDegrees.h"

namespace curaengine::infill
{
class LightningGenerator;
class SierpinskiFillProvider;
}

namespace curaengine
{



class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content

    std::vector<types::AngleDegrees> support_infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<types::AngleDegrees> support_infill_angles_layer_0; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<types::AngleDegrees> support_roof_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<types::AngleDegrees> support_bottom_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer

    std::vector<SupportLayer> supportLayers;
    std::shared_ptr<infill::SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern
    std::shared_ptr<infill::LightningGenerator> lightning_generator; //!< Pre-computed structure for Lightning type infill

    SupportStorage();
    ~SupportStorage();
};

} // namespace curaengine
