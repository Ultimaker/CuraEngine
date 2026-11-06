// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/PositionBasedScoringCriterion.h"

#include "types/geometry/PointsSet.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

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

} // namespace curaengine::types::scoring
