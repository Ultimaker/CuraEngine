// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H
#define PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H

#include "ProgressEstimator.h"

#include <vector>

namespace cura
{


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

} // namespace cura

#endif // PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H