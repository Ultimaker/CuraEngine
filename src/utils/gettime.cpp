// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/gettime.h"

#include <spdlog/stopwatch.h>

namespace cura
{

TimeKeeper::TimeKeeper()
{
}

double TimeKeeper::restart()
{
    double ret = watch.elapsed().count();
    watch.reset();
    return ret;
}

void TimeKeeper::registerTime(const std::string& stage, double threshold)
{
    double duration = restart();
    if (duration >= threshold)
    {
        registered_times.emplace_back(RegisteredTime{ stage, duration });
    }
}

} // namespace cura
