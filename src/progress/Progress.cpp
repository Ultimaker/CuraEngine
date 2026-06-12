// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "progress/Progress.h"

#include <algorithm>
#include <cassert>
#include <optional>

#include <fmt/chrono.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get the communication channel to send progress through.
#include "communication/Communication.h" //To send progress through the communication channel.
#include "utils/gettime.h"

namespace cura
{
std::array<double, N_PROGRESS_STAGES> Progress::accumulated_times = { -1 };
double Progress::total_timing = -1;
std::optional<LayerIndex> Progress::first_skipped_layer{};

double Progress::calcOverallProgress(Stage stage, double stage_progress)
{
    stage_progress = std::clamp(stage_progress, 0.0, 1.0);
    return (accumulated_times.at(static_cast<size_t>(stage)) + stage_progress * times.at(static_cast<size_t>(stage))) / total_timing;
}

void Progress::init()
{
    double accumulated_time = 0;
    for (size_t stage = 0; stage < N_PROGRESS_STAGES; stage++)
    {
        accumulated_times.at(static_cast<size_t>(stage)) = accumulated_time;
        accumulated_time += times.at(static_cast<size_t>(stage));
    }
    total_timing = accumulated_time;
}

void Progress::messageProgress(Progress::Stage stage, int progress_in_stage, int progress_in_stage_max)
{
    double percentage = calcOverallProgress(stage, static_cast<double>(progress_in_stage / static_cast<double>(progress_in_stage_max)));
    Application::getInstance().communication_->sendProgress(percentage);
}

void Progress::messageProgressStage(Progress::Stage stage, TimeKeeper* time_keeper)
{
    if (time_keeper != nullptr)
    {
        if (static_cast<int>(stage) > 0)
        {
            spdlog::info("Progress: {} processed in {}", names.at(static_cast<size_t>(stage) - 1), std::chrono::duration<double>(time_keeper->restart()));
        }
        else
        {
            time_keeper->restart();
        }

        if (static_cast<int>(stage) < static_cast<int>(Stage::FINISH))
        {
            spdlog::info("Starting {}...", names.at(static_cast<size_t>(stage)));
        }
    }
}

void Progress::messageProgressLayer(const LayerIndex layer_nr, const size_t total_layers, const TimeKeeper& time_keeper, const std::chrono::milliseconds skip_threshold)
{
    if (time_keeper.getTotalDuration() < skip_threshold)
    {
        if (! first_skipped_layer)
        {
            first_skipped_layer = layer_nr;
        }
    }
    else
    {
        if (first_skipped_layer)
        {
            spdlog::info("Skipped time reporting for layers [{}...{}]", first_skipped_layer.value(), layer_nr);
            first_skipped_layer.reset();
        }

        messageProgress(Stage::EXPORT, std::max(layer_nr.value, LayerIndex::value_type(0)) + 1, total_layers);

        time_keeper.logRegisteredTimes(fmt::format("Layer export [{}]", layer_nr));
    }
}

} // namespace cura
