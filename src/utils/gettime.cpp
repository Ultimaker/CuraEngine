// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/gettime.h"

#include <fmt/chrono.h>
#include <range/v3/view/enumerate.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

namespace cura
{
TimeKeeper::TimeKeeper()
{
}

std::chrono::milliseconds TimeKeeper::restart()
{
    std::chrono::milliseconds ret = watch.elapsed_ms();
    watch.reset();
    return ret;
}

void TimeKeeper::registerTime(const std::string& stage, const std::chrono::milliseconds threshold, const std::optional<std::chrono::milliseconds> duration)
{
    const std::chrono::milliseconds measured_duration = restart();
    const std::chrono::milliseconds actual_duration = duration.value_or(measured_duration);
    if (actual_duration >= threshold)
    {
        registered_times.emplace_back(RegisteredTime{ stage, actual_duration });
    }
}

void TimeKeeper::end()
{
    total_duration = watch_total.elapsed_ms();
}

std::chrono::milliseconds TimeKeeper::getTotalDuration() const
{
    return total_duration;
}

void TimeKeeper::logRegisteredTimes(const std::string& global_desc) const
{
    spdlog::info("┌ {} processed in {}", global_desc, std::chrono::duration<double>(total_duration));

    auto iterator_max_size = std::max_element(
        registered_times.begin(),
        registered_times.end(),
        [](const RegisteredTime& time1, const RegisteredTime& time2)
        {
            return time1.stage.size() < time2.stage.size();
        });
    if (iterator_max_size != registered_times.end())
    {
        size_t padding = iterator_max_size->stage.size();

        for (const auto& [index, time] : registered_times | ranges::views::enumerate)
        {
            spdlog::info(
                "{}── {}:{} {}",
                index < registered_times.size() - 1 ? "├" : "└",
                time.stage,
                std::string(padding - time.stage.size(), ' '),
                std::chrono::duration<double>(time.duration));
        }
    }
}

} // namespace cura
