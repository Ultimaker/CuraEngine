/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "TimeMaterialEstimates.h"

namespace cura
{

TimeMaterialEstimates TimeMaterialEstimates::operator-(const TimeMaterialEstimates& other)
{
    return TimeMaterialEstimates(extrude_time - other.extrude_time,unretracted_travel_time - other.unretracted_travel_time,retracted_travel_time - other.retracted_travel_time,material - other.material);
}

TimeMaterialEstimates& TimeMaterialEstimates::operator-=(const TimeMaterialEstimates& other)
{
    extrude_time -= other.extrude_time;
    unretracted_travel_time -= other.unretracted_travel_time;
    retracted_travel_time -= other.retracted_travel_time;
    material -= other.material;
    return *this;
}

}//namespace cura
