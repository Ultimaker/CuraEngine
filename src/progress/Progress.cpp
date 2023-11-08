// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "progress/Progress.h"

#include <cassert>
#include <optional>

#include <range/v3/view/enumerate.hpp>
#include <spdlog/spdlog.h>

#include "Application.h" //To get the communication channel to send progress through.
#include "communication/Communication.h" //To send progress through the communication channel.
#include "utils/gettime.h"

namespace cura
{

double Progress::times[] = {
    0.0, // START   = 0,
    5.269, // SLICING = 1,
    1.533, // PARTS   = 2,
    71.811, // INSET_SKIN = 3
    51.009, // SUPPORT = 4,
    154.62, // EXPORT  = 5,
    0.1 // FINISH  = 6
};
std::string Progress::names[] = { "start", "slice", "layerparts", "inset+skin", "support", "export", "process" };

double Progress::accumulated_times[N_PROGRESS_STAGES] = { -1 };
double Progress::total_timing = -1;
std::optional<LayerIndex> Progress::first_skipped_layer{};

float Progress::calcOverallProgress(Stage stage, float stage_progress)
{
    assert(stage_progress <= 1.0);
    assert(stage_progress >= 0.0);
    return (accumulated_times[(int)stage] + stage_progress * times[(int)stage]) / total_timing;
}

void Progress::init()
{
    double accumulated_time = 0;
    for (int stage = 0; stage < N_PROGRESS_STAGES; stage++)
    {
        accumulated_times[(int)stage] = accumulated_time;
        accumulated_time += times[(int)stage];
    }
    total_timing = accumulated_time;
}

void Progress::messageProgress(Progress::Stage stage, int progress_in_stage, int progress_in_stage_max)
{
    float percentage = calcOverallProgress(stage, float(progress_in_stage) / float(progress_in_stage_max));
    Application::getInstance().communication->sendProgress(percentage);
}

void Progress::messageProgressStage(Progress::Stage stage, TimeKeeper* time_keeper)
{
    if (time_keeper)
    {
        if ((int)stage > 0)
        {
            spdlog::info("Progress: {} accomplished in {:03.3f}s", names[(int)stage - 1], time_keeper->restart());
        }
        else
        {
            time_keeper->restart();
        }

        if ((int)stage < (int)Stage::FINISH)
        {
            spdlog::info("Starting {}...", names[(int)stage]);
        }
    }
}

void Progress::messageProgressLayer(LayerIndex layer_nr, size_t total_layers, double total_time, const TimeKeeper::RegisteredTimes& stages, double skip_threshold)
{
    if (total_time < skip_threshold)
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
            spdlog::info("Skipped time reporting for layers [{}...{}]", first_skipped_layer.value().value, layer_nr.value);
            first_skipped_layer.reset();
        }

        messageProgress(Stage::EXPORT, std::max(layer_nr.value, LayerIndex::value_type(0)) + 1, total_layers);

        spdlog::info("┌ Layer export [{}] accomplished in {:03.3f}s", layer_nr.value, total_time);

        size_t padding = 0;
        auto iterator_max_size = std::max_element(
            stages.begin(),
            stages.end(),
            [](const TimeKeeper::RegisteredTime& time1, const TimeKeeper::RegisteredTime& time2)
            {
                return time1.stage.size() < time2.stage.size();
            });
        if (iterator_max_size != stages.end())
        {
            padding = iterator_max_size->stage.size();

            for (auto [index, time] : stages | ranges::view::enumerate)
            {
                spdlog::info("{}── {}:{} {:03.3f}s", index < stages.size() - 1 ? "├" : "└", time.stage, std::string(padding - time.stage.size(), ' '), time.duration);
            }
        }
    }
}

} // namespace cura
