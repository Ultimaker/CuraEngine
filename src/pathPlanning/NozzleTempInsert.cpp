//Copyright (C) 2016 Ultimaker
//Released under terms of the AGPLv3 License

#include "NozzleTempInsert.h"

namespace cura
{

NozzleTempInsert::NozzleTempInsert(unsigned int path_idx, int extruder, double temperature, bool wait, double time_after_path_start)
: path_idx(path_idx)
, time_after_path_start(time_after_path_start)
, extruder(extruder)
, temperature(temperature)
, wait(wait)
{
    assert(temperature != 0 && temperature != -1 && "Temperature command must be set!");
}

void NozzleTempInsert::write(GCodeExport& gcode)
{
    gcode.writeTemperatureCommand(extruder, temperature, wait);
}

}