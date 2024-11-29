// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "plan_export/GCodeExporter.h"
#include "path_planning/NozzleTempInsert.h"

namespace cura
{

void NozzleTempInsert::write(GCodeExporter& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

} // namespace cura