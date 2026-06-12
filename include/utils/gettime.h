// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GETTIME_H
#define GETTIME_H

#include <chrono>
#include <string>
#include <vector>

#include <spdlog/stopwatch.h>

using namespace std::chrono_literals;

namespace cura
{

class TimeKeeper
{
public:
    struct RegisteredTime
    {
        std::string stage;
        std::chrono::milliseconds duration;
    };

    using RegisteredTimes = std::vector<RegisteredTime>;

private:
    spdlog::stopwatch watch;
    spdlog::stopwatch watch_total;
    RegisteredTimes registered_times;
    std::chrono::milliseconds total_duration;

public:
    TimeKeeper();

    std::chrono::milliseconds restart();

    void registerTime(const std::string& stage, std::chrono::milliseconds threshold = 10ms);

    const RegisteredTimes& getRegisteredTimes() const
    {
        return registered_times;
    }

    void end();

    std::chrono::milliseconds getTotalDuration() const;

    void logRegisteredTimes(const std::string& global_desc) const;
};

} // namespace cura
#endif // GETTIME_H
