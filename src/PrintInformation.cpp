// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrintInformation.h"

#include "LayerPlan.h"
#include "sliceDataStorage.h"


void cura::PrintInformation::updateWithLayer(const SliceDataStorage& storage, const LayerPlan* layer_plan)
{
    if (! initial_extruder_nr.has_value())
    {
        initial_extruder_nr = layer_plan->findInitialExtruderNr();
    }

    if (layer_plan->getLayerNr() == 0)
    {
        constexpr bool include_support = true;
        constexpr bool include_prime_tower = true;
        constexpr bool external_polys_only = true;
        constexpr int extruder_nr = -1;
        constexpr bool include_models = true;
        initial_layer_bb.include(AABB(storage.getLayerOutlines(layer_plan->getLayerNr(), include_support, include_prime_tower, external_polys_only, extruder_nr, include_models)));
    }
}
