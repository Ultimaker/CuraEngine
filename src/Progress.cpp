/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "Progress.h"

#include "commandSocket.h"
#include "utils/gettime.h"

namespace cura {
    
double Progress::times [] = 
{ 
    0.0, 
    5.269, 
    1.533, 
    22.953,
    51.009, 
    48.858, 
    154.62, 
    0.1 
};
std::string Progress::names [] = 
{
    "start",
    "slice",
    "layerparts",
    "inset",
    "support",
    "skin",
    "export",
    "process"
};

    
double Progress::accumulated_times [N_PROGRESS_STAGES] = {-1};
double Progress::total_timing = -1;

/*
const Progress::Stage Progress::stages[] = 
{ 
    Progress::Stage::START, 
    Progress::Stage::SLICING, 
    Progress::Stage::PARTS, 
    Progress::Stage::INSET, 
    Progress::Stage::SUPPORT, 
    Progress::Stage::SKIN, 
    Progress::Stage::EXPORT, 
    Progress::Stage::FINISH 
};
*/

float Progress::calcOverallProgress(Stage stage, float stage_progress)
{
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

void Progress::messageProgress(Progress::Stage stage, int progress_in_stage, int progress_in_stage_max, CommandSocket* command_socket)
{
    float percentage = calcOverallProgress(stage, float(progress_in_stage) / float(progress_in_stage_max));
    if (command_socket)
    {
        command_socket->sendProgress(percentage);
    }
    
    logProgress(names[(int)stage].c_str(), progress_in_stage, progress_in_stage_max, percentage);
}

void Progress::messageProgressStage(Progress::Stage stage, TimeKeeper* time_keeper, CommandSocket* command_socket)
{
    if (command_socket)
    {
        command_socket->sendProgressStage(stage);
    }
    
    if (time_keeper)
    {
        if ((int)stage > 0)
        {
            log("Progress: %s accomplished in %5.3fs\n", names[(int)stage-1].c_str(), time_keeper->restart());
        }
        else
        {
            time_keeper->restart();
        }
        
        if ((int)stage < (int)Stage::FINISH)
            log("Starting %s...\n", names[(int)stage].c_str());
    }
}

}// namespace cura