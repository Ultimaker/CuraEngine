#ifndef PROGRESS_ESTIMATOR_H
#define PROGRESS_ESTIMATOR_H

#include <vector>

namespace cura
{
/*
 * ProgressEstimator is a finger-tree with ProgressEstimatorLinear as leaves.
 * 
 * Each (non-leaf) node consists of a ProgressStageEstimator which consists of several stages.
 * 
 * The structure of this tree is an oversimplification of the call graph of CuraEngine.
 * 
 */
    
class ProgressEstimator
{
public:
    virtual double progress(int current_step) = 0;
    virtual ~ProgressEstimator()
    {
    }
};

class ProgressEstimatorLinear : public ProgressEstimator
{
    unsigned int total_steps;
public:
    ProgressEstimatorLinear(unsigned int total_steps)
    : total_steps(total_steps)
    {
    }
    double progress(int current_step)
    {
        return double(current_step) / double(total_steps);
    }
};

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

#endif // PROGRESS_ESTIMATOR_H