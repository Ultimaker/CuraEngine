// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/RandomScoringCriterion.h"

#include "utils/math.h"

namespace curaengine::utils::scoring
{

RandomScoringCriterion::RandomScoringCriterion()
{
}

double RandomScoringCriterion::computeScore(const size_t /*candidate_index*/) const
{
    return utils::randf<double>();
}

} // namespace curaengine::utils::scoring
