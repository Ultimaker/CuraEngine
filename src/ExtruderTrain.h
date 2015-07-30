#ifndef EXTRUDER_TRAIN_H
#define EXTRUDER_TRAIN_H

#include "settings.h"

namespace cura 
{
    
class ExtruderTrain : public SettingsBase
{
    int extruder_nr;
public:
    int getExtruderNr() { return extruder_nr; }
    
    ExtruderTrain(SettingsBase* settings, int extruder_nr)
    : SettingsBase(settings)
    , extruder_nr(extruder_nr)
    { }
    
};

}//namespace cura
#endif // EXTRUDER_TRAIN_H