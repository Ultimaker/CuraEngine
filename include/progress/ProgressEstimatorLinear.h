// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H
#define PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H

#include <vector>

#include "ProgressEstimator.h"

namespace cura
{


class ProgressEstimatorLinear : public ProgressEstimator
{
    unsigned int total_steps_;

public:
    ProgressEstimatorLinear(unsigned int total_steps)
        : total_steps_(total_steps)
    {
    }
    double progress(int current_step)
    {
        return double(current_step) / double(total_steps_);
    }
};

} // namespace cura

#endif // PROGRESS_PROGRESS_ESTIMATOR_LINEAR_H
