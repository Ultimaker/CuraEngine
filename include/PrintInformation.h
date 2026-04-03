// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRINTINFORMATION_H
#define PRINTINFORMATION_H

#include <optional>
#include <string>
#include <vector>

#include "utils/AABB.h"

namespace cura
{

class LayerPlan;
class SliceDataStorage;

/*! \brief Contains the end-of-print info about used material for an extruder */
struct ExtruderPrintInformation
{
    float filament_amount{}; // Material volume in mm3
    float filament_length{}; // Filament length in m
    float filament_weight{}; // Filament weight in grams
    float filament_cost{}; // Filament cost (unspecified currency)
    std::string material_name; // Material full name
};

struct PrintInformation
{
    std::vector<std::optional<ExtruderPrintInformation>> extruders_info; // One per extruder, but no value if the extruder is unused
    std::optional<size_t> initial_extruder_nr;
    AABB initial_layer_bb;

    void updateWithLayer(const LayerPlan* layer_plan);
};

} // namespace cura

#endif // PRINTINFORMATION_H
