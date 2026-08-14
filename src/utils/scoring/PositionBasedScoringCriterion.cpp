// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/PositionBasedScoringCriterion.h"

#include "geometry/PointsSet.h"


namespace cura
{

PositionBasedScoringCriterion::PositionBasedScoringCriterion(const PointsSet& points)
    : points_(points)
{
}

double PositionBasedScoringCriterion::computeScore(const size_t candidate_index) const
{
    return computeScore(points_.at(candidate_index));
}

double PositionBasedScoringCriterion::computeScore(const Point2LL& candidate_position) const
{
    return 0.0;
}

} // namespace cura
