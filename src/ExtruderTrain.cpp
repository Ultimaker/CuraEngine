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

bool ExtruderTrain::getIsUsed() const
{
    return is_used;
}

void ExtruderTrain::setIsUsed(bool used)
{
    is_used = used;
}


}//namespace cura