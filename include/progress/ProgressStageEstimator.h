// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "ProgressEstimator.h"

namespace curaengine
{

/*!
 * A staged progress estimator which estimates each stage to have different times.
 */
class ProgressStageEstimator : public ProgressEstimator
{
    struct ProgressStage
    {
        double relative_estimated_time_;
        ProgressEstimator* stage_;

        ProgressStage(double relative_estimated_time)
            : relative_estimated_time_(relative_estimated_time)
            , stage_(nullptr)
        {
        }
    };

protected:
    std::vector<ProgressStage> stages_;
    double total_estimated_time_;

private:
    double accumulated_estimate_;
    int current_stage_idx_;

public:
    ProgressStageEstimator(std::vector<double>& relative_time_estimates);

    double progress(int current_step);

    /*!
     *
     * \warning This class is responsible for deleting the \p stage
     *
     */
    void nextStage(ProgressEstimator* stage);

    ~ProgressStageEstimator();
};

} // namespace curaengine
