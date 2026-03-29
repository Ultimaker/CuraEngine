// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRINTINFORMATION_H
#define PRINTINFORMATION_H

#include <optional>
#include <string>
#include <vector>

struct ExtruderPrintInformation
{
    float filament_amount{};
    float filament_length{};
    float filament_weight{};
    float filament_cost{};
    std::string material_name;
};

// One per extruder, but no value if the extruder is unused
using PrintInformation = std::vector<std::optional<ExtruderPrintInformation>>;

#endif // PRINTINFORMATION_H
