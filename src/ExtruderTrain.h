/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef EXTRUDER_TRAIN_H
#define EXTRUDER_TRAIN_H

#include "settings/settings.h"

namespace cura 
{
    
class ExtruderTrain : public SettingsBase
{
    int extruder_nr;
    bool is_used = false; //!< whether this extruder train is (probably) used during printing the current meshgroup
public:
    int getExtruderNr();

    bool getIsUsed() const; //!< return whether this extruder train is (probably) used during printing the current meshgroup
    void setIsUsed(bool used); //!< set whether this extruder train is (probably) used during printing the current meshgroup

    ExtruderTrain(SettingsBaseVirtual* settings, int extruder_nr);

};

}//namespace cura
#endif // EXTRUDER_TRAIN_H