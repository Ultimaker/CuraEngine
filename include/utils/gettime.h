// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GETTIME_H
#define GETTIME_H

#include <chrono>
#include <string>
#include <vector>

#include <spdlog/stopwatch.h>

namespace cura
{

class TimeKeeper
{
public:
    struct RegisteredTime
    {
        std::string stage;
        double duration;
    };

    using RegisteredTimes = std::vector<RegisteredTime>;

private:
    spdlog::stopwatch watch;
    double start_time;
    RegisteredTimes registered_times;

public:
    TimeKeeper();

    double restart();

    void registerTime(const std::string& stage, double threshold = 0.01);

    const RegisteredTimes& getRegisteredTimes() const
    {
        return registered_times;
    }
};

} // namespace cura
#endif // GETTIME_H
