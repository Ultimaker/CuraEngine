// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/DistanceScoringCriterion.h"

#include "geometry/PointsSet.h"


namespace cura
{

DistanceScoringCriterion::DistanceScoringCriterion(const PointsSet& points, const Point2LL& target_pos)
    : points_(points)
    , target_pos_(target_pos)
{
}

double DistanceScoringCriterion::computeScore(const size_t candidate_index) const
{
    const Point2LL& candidate_position = points_.at(candidate_index);

    // Fixed divider for shortest distances computation. The divider should be set so that the minimum encountered
    // distance gives a score very close to 1.0, and a medium-far distance gives a score close to 0.5
    constexpr double distance_divider = 20.0;

    // Use actual (non-squared) distance to ensure a proper scoring distribution
    const double distance = vSizeMM(candidate_position - target_pos_);

    // Use reciprocal function to normalize distance score decreasingly
    return 1.0 / (1.0 + (distance / distance_divider));
}

} // namespace cura
