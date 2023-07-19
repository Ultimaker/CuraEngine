// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

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

} // namespace cura