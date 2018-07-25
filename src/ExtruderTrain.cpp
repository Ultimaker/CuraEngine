/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ExtruderTrain.h"

namespace cura 
{
size_t ExtruderTrain::getExtruderNr()
{
    return extruder_nr;
}
ExtruderTrain::ExtruderTrain(SettingsBaseVirtual* settings, const size_t extruder_nr)
: SettingsBase(settings)
, extruder_nr(extruder_nr)
{
}

ExtruderTrain::ExtruderTrain(const size_t extruder_nr, Settings* parent_settings) : extruder_nr(extruder_nr)
{
    settings.setParent(parent_settings);
}

}//namespace cura