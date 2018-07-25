//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Slice.h"

namespace cura
{

Slice::Slice() {}

void Slice::compute(std::string& output_gcode)
{
    output_gcode = ";TODO";
}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

}