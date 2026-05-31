// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrintInformation.h"

#include "LayerPlan.h"
#include "sliceDataStorage.h"


void cura::PrintInformation::updateWithLayer(const LayerPlan* layer_plan)
{
    // Find the first layer that has an extrusion (most likely layer 0) and keep the very initial extruder nr
    if (! initial_extruder_nr.has_value())
    {
        initial_extruder_nr = layer_plan->findInitialExtruderNr();
    }

    if (layer_plan->getLayerNr() == 0)
    {
        initial_layer_bb.include(layer_plan->calculateExtrusionBoundingBox());
    }
}
