// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "pathPlanning/NozzleTempInsert.h"

#include "gcode_export/GCodeExport.h"

namespace curaengine
{

void NozzleTempInsert::write(GCodeExport& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

} // namespace curaengine