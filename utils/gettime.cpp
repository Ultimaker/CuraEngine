/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "gettime.h"

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
