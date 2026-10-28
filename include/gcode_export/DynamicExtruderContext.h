// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <variant>

namespace curaengine::gcode_export
{

enum class DynamicExtruderContext
{
    Global, // No extruder for this context, only use the global variables
    Initial, // Use the initial extruder, which will be resolved at the last moment
};

// Contains either a dynamic extruder context, or a fixed extruder number that we know in advance
using ResolvingExtruderContext = std::variant<DynamicExtruderContext, size_t>;

} // namespace curaengine::gcode_export
