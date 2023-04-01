//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/gettime.h"

namespace cura
{
    
TimeKeeper::TimeKeeper()
{
    restart();
}

double TimeKeeper::restart()
{
    double ret = getTime() - startTime;
    startTime = getTime();
    return ret;
}

}//namespace cura