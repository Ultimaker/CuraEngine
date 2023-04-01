// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcodeExport.h"
#include "pathPlanning/NozzleTempInsert.h"

namespace cura
{

void NozzleTempInsert::write(GCodeExport& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

} // namespace cura