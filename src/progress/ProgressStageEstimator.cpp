/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ProgressStageEstimator.h"


namespace cura
{


ProgressStageEstimator::ProgressStageEstimator(std::vector< double >& relative_time_estimates)
: total_estimated_time(0)
, accumulated_estimate(0)
, current_stage_idx(-1)
{
    stages.reserve(relative_time_estimates.size());
    for (double relative_estimated_time : relative_time_estimates)
    {
        stages.emplace_back(relative_estimated_time);
        total_estimated_time += relative_estimated_time;
    }
}

ProgressStageEstimator::~ProgressStageEstimator()
{
    for (ProgressStage& stage : stages)
    {
        delete stage.stage;
    }
}

double ProgressStageEstimator::progress(int current_step)
{
    ProgressStage& current_stage = stages[current_stage_idx];
    return (accumulated_estimate + current_stage.stage->progress(current_step) * current_stage.relative_estimated_time) / total_estimated_time;
}

void ProgressStageEstimator::nextStage(ProgressEstimator* stage)
{
    if (current_stage_idx >= int(stages.size()) - 1)
    {
        return;
    }
    if (current_stage_idx >= 0)
    {
        ProgressStage& current_stage = stages[current_stage_idx];
        accumulated_estimate += current_stage.relative_estimated_time;
    }
    current_stage_idx++;
    stages[current_stage_idx].stage = stage;
}


} // namespace cura
