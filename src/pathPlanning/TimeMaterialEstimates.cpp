/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "TimeMaterialEstimates.h"

namespace cura
{

TimeMaterialEstimates::TimeMaterialEstimates(double extrude_time, double unretracted_travel_time, double retracted_travel_time, double material)
: extrude_time(extrude_time)
, unretracted_travel_time(unretracted_travel_time)
, retracted_travel_time(retracted_travel_time)
, material(material)
{
}

TimeMaterialEstimates::TimeMaterialEstimates()
: extrude_time(0.0)
, unretracted_travel_time(0.0)
, retracted_travel_time(0.0)
, material(0.0)
{
}

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

TimeMaterialEstimates TimeMaterialEstimates::operator+(const TimeMaterialEstimates& other)
{
    return TimeMaterialEstimates(extrude_time+other.extrude_time, unretracted_travel_time+other.unretracted_travel_time, retracted_travel_time+other.retracted_travel_time, material+other.material);
}

TimeMaterialEstimates& TimeMaterialEstimates::operator+=(const TimeMaterialEstimates& other)
{
    extrude_time += other.extrude_time;
    unretracted_travel_time += other.unretracted_travel_time;
    retracted_travel_time += other.retracted_travel_time;
    material += other.material;
    return *this;
}

double TimeMaterialEstimates::getExtrudeTime() const
{
    return extrude_time;
}

double TimeMaterialEstimates::getMaterial() const
{
    return material;
}

double TimeMaterialEstimates::getTotalTime() const
{
    return extrude_time + unretracted_travel_time + retracted_travel_time;
}

double TimeMaterialEstimates::getTotalUnretractedTime() const
{
    return extrude_time + unretracted_travel_time;
}

double TimeMaterialEstimates::getTravelTime() const
{
    return retracted_travel_time + unretracted_travel_time;
}

void TimeMaterialEstimates::reset()
{
    extrude_time = 0.0;
    unretracted_travel_time = 0.0;
    retracted_travel_time = 0.0;
    material = 0.0;
}

}//namespace cura
