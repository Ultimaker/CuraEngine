// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/RandomScoringCriterion.h"

#include "seam/seam_corners.h"
#include "utils/math.h"

namespace curaengine::types::scoring
{

template<typename T>
double RandomScoringCriterion<T>::computeScore(const T& /*element*/) const
{
    return utils::randf<double>();
}

template class RandomScoringCriterion<seam::SeamCandidate>;

} // namespace curaengine::types::scoring
