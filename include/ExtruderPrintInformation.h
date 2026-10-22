// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <string>

namespace cura
{
/*! \brief Contains the end-of-print info about used material for an extruder */
struct ExtruderPrintInformation
{
    float filament_amount{}; // Material volume in mm3
    float filament_length{}; // Filament length in m
    float filament_weight{}; // Filament weight in grams
    float filament_cost{}; // Filament cost (unspecified currency)
    std::string material_name; // Material full name
    std::string material_guid; // Material guid
};

} // namespace cura
