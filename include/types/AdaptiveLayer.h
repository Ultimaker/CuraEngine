// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "utils/Coord_t.h"

namespace curaengine::types
{

class AdaptiveLayer
{
public:
    /*!
     * Height of the layer in microns.
     */
    ::curaengine::utils::coord_t layer_height_;

    /*!
     * The absolute z position of the layer.
     */
    ::curaengine::utils::coord_t z_position_;

    /*!
     * Temperature to use for this layer.
     */
    int temperature_;

    explicit AdaptiveLayer(const ::curaengine::utils::coord_t layer_height);
};

} // namespace curaengine::types
