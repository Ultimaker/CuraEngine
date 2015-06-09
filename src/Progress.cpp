/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "Progress.h"

#include "commandSocket.h"

namespace cura {
    
double Progress::times [] = { 0.0, 5.269, 1.533, 22.953, 51.009, 48.858, 154.62, 0.0 };
double Progress::accumulated_times [] = {-1};
double Progress::totalTiming = -1;

const Progress::Stage Progress::stages[] = 
{ 
    Progress::Stage::START, 
    Progress::Stage::SLICING, 
    Progress::Stage::PARTS, 
    Progress::Stage::INSET, 
    Progress::Stage::SKIN, 
    Progress::Stage::SUPPORT, 
    Progress::Stage::EXPORT, 
    Progress::Stage::FINISH 
};

float Progress::calcOverallProgress(Stage stage, float stage_progress)
{
    return ( accumulated_times[(int)stage] + stage_progress / times[(int)stage] ) / totalTiming;
}


void Progress::init()
{
    double accumulated_time = 0;
    for (int stage = 0; stage < N_PROGRESS_STAGES; stage++)
    {
        accumulated_times[(int)stage] = accumulated_time;
        accumulated_time += times[(int)stage];
    }
    totalTiming = accumulated_time;
}

void Progress::messageProgress(Progress::Stage stage, int progress_in_stage, int progress_in_stage_max, CommandSocket* commandSocket)
{
    if (commandSocket)
    {
        commandSocket->sendProgress(calcOverallProgress(stage, float(progress_in_stage) / float(progress_in_stage_max)));
    }
    switch (stage)
    {
    case Stage::START:
        logProgress("start", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::SLICING:
        logProgress("slice", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::PARTS:
        logProgress("layerparts", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::INSET:
        logProgress("inset", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::SKIN:
        logProgress("skin", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::SUPPORT:
        logProgress("support", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::EXPORT:
        logProgress("export", progress_in_stage, progress_in_stage_max);
        break;
    case Stage::FINISH:
        logProgress("process", progress_in_stage, progress_in_stage_max);
        break;
    }
}

void Progress::messageProgressStage(Progress::Stage stage, CommandSocket* commandSocket)
{
    commandSocket->sendProgressStage(stage);
}



}// namespace cura