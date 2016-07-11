/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef EXTRUDER_TRAIN_H
#define EXTRUDER_TRAIN_H

#include "settings/settings.h"

namespace cura 
{
    
class ExtruderTrain : public SettingsBase
{
    int extruder_nr;
public:
    int getExtruderNr();

    ExtruderTrain(SettingsBaseVirtual* settings, int extruder_nr);

};

}//namespace cura
#endif // EXTRUDER_TRAIN_H