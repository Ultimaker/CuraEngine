// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "ProgressEstimator.h"

namespace curaengine::progress
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

} // namespace curaengine::progress
