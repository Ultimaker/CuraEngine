// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/PositionBasedRankingCriterion.h"

#include "types/geometry/PointsSet.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;

PositionBasedRankingCriterion::PositionBasedRankingCriterion(const PointsSet& points)
    : points_(points)
{
}

uint8_t PositionBasedRankingCriterion::computeRank(const size_t candidate_index) const
{
    return computeRank(points_.at(candidate_index));
}

} // namespace curaengine::types::scoring
