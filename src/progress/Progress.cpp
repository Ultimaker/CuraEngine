// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <cassert>

#include <spdlog/spdlog.h>

#include "Application.h" //To get the communication channel to send progress through.
#include "communication/Communication.h" //To send progress through the communication channel.
#include "progress/Progress.h"
#include "utils/gettime.h"

namespace cura
{

double Progress::times[] = {
    0.0,    // START   = 0, 
    5.269,  // SLICING = 1, 
    1.533,  // PARTS   = 2, 
    71.811, // INSET_SKIN = 3
    51.009, // SUPPORT = 4, 
    154.62, // EXPORT  = 5, 
    0.1     // FINISH  = 6
};
std::string Progress::names [] = 
{
    "start",
    "slice",
    "layerparts",
    "inset+skin",
    "support",
    "export",
    "process"
};

double Progress::accumulated_times [N_PROGRESS_STAGES] = {-1};
double Progress::total_timing = -1;

float Progress::calcOverallProgress(Stage stage, float stage_progress)
{
    assert(stage_progress <= 1.0);
    assert(stage_progress >= 0.0);
    return ( accumulated_times[(int)stage] + stage_progress * times[(int)stage] ) / total_timing;
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

    // logProgress(names[(int)stage].c_str(), progress_in_stage, progress_in_stage_max, percentage); FIXME: use different sink
}

void Progress::messageProgressStage(Progress::Stage stage, TimeKeeper* time_keeper)
{
    if (time_keeper)
    {
        if ((int)stage > 0)
        {
            spdlog::info("Progress: {} accomplished in {:3}s", names[(int)stage - 1], time_keeper->restart());
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

}// namespace cura