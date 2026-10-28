// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/RandomScoringCriterion.h"

#include "utils/math.h"

namespace curaengine
{

RandomScoringCriterion::RandomScoringCriterion()
{
}

double RandomScoringCriterion::computeScore(const size_t /*candidate_index*/) const
{
    return curaengine::randf<double>();
}

} // namespace curaengine
