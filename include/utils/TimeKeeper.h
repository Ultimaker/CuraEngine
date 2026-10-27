// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <chrono>
#include <optional>
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

    void registerTime(const std::string& stage, const std::chrono::milliseconds threshold = 10ms, const std::optional<std::chrono::milliseconds> duration = std::nullopt);

    const RegisteredTimes& getRegisteredTimes() const
    {
        return registered_times;
    }

    void end();

    std::chrono::milliseconds getTotalDuration() const;

    void logRegisteredTimes(const std::string& global_desc) const;
};

} // namespace cura
