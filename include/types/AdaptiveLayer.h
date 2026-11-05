// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "types/Coord_t.h"

namespace curaengine::types
{

class AdaptiveLayer
{
public:
    /*!
     * Height of the layer in microns.
     */
    types::coord_t layer_height_;

    /*!
     * The absolute z position of the layer.
     */
    types::coord_t z_position_;

    /*!
     * Temperature to use for this layer.
     */
    int temperature_;

    explicit AdaptiveLayer(const types::coord_t layer_height);
};

} // namespace curaengine::types
