// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/RandomScoringCriterion.h"

#include "utils/math.h"

namespace curaengine::types::scoring
{

using namespace types;
using namespace utils;

RandomScoringCriterion::RandomScoringCriterion()
{
}

double RandomScoringCriterion::computeScore(const size_t /*candidate_index*/) const
{
    return utils::randf<double>();
}

} // namespace curaengine::types::scoring
