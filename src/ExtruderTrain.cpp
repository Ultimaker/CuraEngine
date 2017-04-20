/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ExtruderTrain.h"

namespace cura
{
int ExtruderTrain::getExtruderNr()
{
    return extruder_nr;
}
ExtruderTrain::ExtruderTrain(SettingsBaseVirtual* settings, int extruder_nr)
: SettingsBase(settings)
, extruder_nr(extruder_nr)
{
}

}//namespace cura