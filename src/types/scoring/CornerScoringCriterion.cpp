// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/CornerScoringCriterion.h"

#include <algorithm>
#include <cmath>


namespace curaengine::types::scoring
{

using namespace types;

CornerScoringCriterion::CornerScoringCriterion(const EZSeamCornerPrefType corner_preference)
{
    switch (corner_preference)
    {
    case EZSeamCornerPrefType::ZSeamCornerPrefInner:
        // Give advantage to concave corners. More advantage for sharper corners.
        concave_angle_weight_ = 1.0;
        convex_angle_weight_ = 0.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefOuter:
        // Give advantage to convex corners. More advantage for sharper corners.
        concave_angle_weight_ = 0.0;
        convex_angle_weight_ = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefAny:
        // Still give sharper corners more advantage.
        concave_angle_weight_ = 1.0;
        convex_angle_weight_ = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefWeighted:
        // Give sharper corners some advantage, but concave sharper corners even more.
        concave_angle_weight_ = 1.0;
        convex_angle_weight_ = 0.5;
        break;
    case EZSeamCornerPrefType::Plugin:
        break;
    }
}

double CornerScoringCriterion::computeScore(const seam::SeamCandidate& candidate) const
{
    // Negative scores are convex (right turning), positive ones are concave (left turning)
    const bool convex = candidate.score < 0.0;
    return std::min(1.0, (convex ? convex_angle_weight_ : concave_angle_weight_) * std::abs(candidate.score));
}

} // namespace curaengine::types::scoring
