//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GETTIME_H
#define GETTIME_H


namespace cura
{

class TimeKeeper
{
private:
    double startTime;
public:
    TimeKeeper();
    
    double restart();
};

}//namespace cura
#endif//GETTIME_H
