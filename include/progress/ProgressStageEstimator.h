// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PROGRESS_PROGRESS_STAGE_ESTIMATOR_H
#define PROGRESS_PROGRESS_STAGE_ESTIMATOR_H

#include <vector>

#include "ProgressEstimator.h"

namespace cura
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

} // namespace cura

#endif // PROGRESS_PROGRESS_STAGE_ESTIMATOR_H
