// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "routing/NozzleTempInsert.h"

#include "gcode/GCodeExport.h"

namespace curaengine::routing
{
using namespace gcode;

void NozzleTempInsert::write(GCodeExport& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

} // namespace curaengine::routing
