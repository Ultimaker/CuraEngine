// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine
{

enum class GCodePartType
{
    Header, // The header, only made of useful comments
    Management, // Management commands, like heating, extruder switching, custom gcode, ...
    Print, // Actual model print commands

};

} // namespace curaengine
