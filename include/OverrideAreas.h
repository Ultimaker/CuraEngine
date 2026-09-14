// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <vector>

#include "PrintSegmentAttributes.h"
#include "geometry/Shape.h"
#include "settings/types/Ratio.h"

namespace cura
{

struct GCodePathConfig;

/*! Represents an area where the extrusion line should be printed with specific settings */
struct OverrideArea
{
    Shape area; // The area where the extrusion lines are to be printed with specific settings
    PrintSegmentAttributes print_attributes; // Extra print attributes to be set for extrusion segments that are printed inside the area
    const GCodePathConfig* config{ nullptr }; // Configuration to be used when printing extrusion segments in this area, or nullptr
    Ratio speed_factor{ 1.0_r }; // Extra speed ratio to be applied when printing extrusion segments in this area, or 1.0 to have no effect
};

// A vector or override areas is to be applied as a stack: the last area takes precedences over the previous one, and so on.
using OverrideAreas = std::vector<OverrideArea>;

} // namespace cura
