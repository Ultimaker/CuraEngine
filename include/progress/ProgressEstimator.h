// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine::progress
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

} // namespace curaengine::progress
