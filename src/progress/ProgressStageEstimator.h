#ifndef PROGRESS_PROGRESS_STAGE_ESTIMATOR_H
#define PROGRESS_PROGRESS_STAGE_ESTIMATOR_H

#include <vector>

#include "ProgressEstimator.h"

namespace cura
{

/*!
 * 
 */
class ProgressStageEstimator : public ProgressEstimator
{
    struct ProgressStage
    {
        double relative_estimated_time;
        ProgressEstimator* stage;
        ProgressStage(double relative_estimated_time)
        : relative_estimated_time(relative_estimated_time)
        , stage(nullptr)
        {
        }
        
    };
    
protected:
    std::vector<ProgressStage> stages;
    double total_estimated_time;
    
private:
    double accumulated_estimate;
    int current_stage_idx;
    
public:
    ProgressStageEstimator(std::vector<double>& relative_time_estimates)
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
    
    double progress(int current_step)
    {
        ProgressStage& current_stage = stages[current_stage_idx];
        return (accumulated_estimate + current_stage.stage->progress(current_step) * current_stage.relative_estimated_time) / total_estimated_time;
    }
    
    /*!
     * 
     * \warning This class is responsible for deleting the \p stage
     * 
     */
    void nextStage(ProgressEstimator* stage)
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
    
    ~ProgressStageEstimator()
    {
        for (ProgressStage& stage : stages)
        {
            delete stage.stage;
        }
    }
};

} // namespace cura

#endif // PROGRESS_PROGRESS_STAGE_ESTIMATOR_H