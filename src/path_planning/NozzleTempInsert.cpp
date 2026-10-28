// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/NozzleTempInsert.h"

#include "gcode_export/GCodeExport.h"

using namespace curaengine::gcode_export;

namespace curaengine::path_planning
{

void NozzleTempInsert::write(GCodeExport& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

} // namespace curaengine::path_planning
