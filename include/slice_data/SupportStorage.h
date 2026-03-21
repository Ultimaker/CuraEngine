// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SUPPORTSTORAGE_H
#define SLICEDATA_SUPPORTSTORAGE_H

#include "settings/types/Angle.h" //Infill angles.
#include "slice_data/SupportLayer.h"

namespace cura
{

class SierpinskiFillProvider;

class SupportStorage
{
public:
    bool generated{false}; //!< whether generateSupportGrid(.) has completed (successfully)

    int layer_nr_max_filled_layer{-1}; //!< the layer number of the uppermost layer with content

    std::vector<AngleDegrees> support_infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_infill_angles_layer_0; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_roof_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_bottom_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer

    std::vector<SupportLayer> supportLayers;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    SupportStorage() = default;
    ~SupportStorage() = default;
};

} // namespace cura

#endif
