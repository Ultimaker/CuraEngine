//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"

namespace cura 
{

ExtruderTrain::ExtruderTrain(const size_t extruder_nr, Settings* parent_settings) : extruder_nr(extruder_nr)
{
    settings.setParent(parent_settings);
}

}//namespace cura