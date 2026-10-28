// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/scoring/ScoringCriterion.h"

namespace curaengine::utils::scoring
{

/*!
 * Criterion that will give a random score whatever the element is.
 */
class RandomScoringCriterion : public ScoringCriterion
{
public:
    explicit RandomScoringCriterion();

    virtual double computeScore(const size_t candidate_index) const override;
};

} // namespace curaengine::utils::scoring
