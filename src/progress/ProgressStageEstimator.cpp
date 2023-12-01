// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "progress/ProgressStageEstimator.h"


namespace cura
{


ProgressStageEstimator::ProgressStageEstimator(std::vector<double>& relative_time_estimates)
    : total_estimated_time_(0)
    , accumulated_estimate_(0)
    , current_stage_idx_(-1)
{
    stages_.reserve(relative_time_estimates.size());
    for (double relative_estimated_time : relative_time_estimates)
    {
        stages_.emplace_back(relative_estimated_time);
        total_estimated_time_ += relative_estimated_time;
    }
}

ProgressStageEstimator::~ProgressStageEstimator()
{
    for (ProgressStage& stage : stages_)
    {
        delete stage.stage_;
    }
}

double ProgressStageEstimator::progress(int current_step)
{
    ProgressStage& current_stage = stages_[current_stage_idx_];
    return (accumulated_estimate_ + current_stage.stage_->progress(current_step) * current_stage.relative_estimated_time_) / total_estimated_time_;
}

void ProgressStageEstimator::nextStage(ProgressEstimator* stage)
{
    if (current_stage_idx_ >= int(stages_.size()) - 1)
    {
        return;
    }
    if (current_stage_idx_ >= 0)
    {
        ProgressStage& current_stage = stages_[current_stage_idx_];
        accumulated_estimate_ += current_stage.relative_estimated_time_;
    }
    current_stage_idx_++;
    stages_[current_stage_idx_].stage_ = stage;
}


} // namespace cura
