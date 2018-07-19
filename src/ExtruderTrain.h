//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXTRUDER_TRAIN_H
#define EXTRUDER_TRAIN_H

#include "settings/Settings.h"

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
