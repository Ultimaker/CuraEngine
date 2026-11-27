// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/ExclusionAreaRankingCriterion.h"

#include "types/geometry/Shape.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

ExclusionAreaRankingCriterion::ExclusionAreaRankingCriterion(const PointsSet& points, const Shape& exclusion_area)
    : PositionBasedRankingCriterion(points)
    , exclusion_area_(exclusion_area)
{
}

uint8_t ExclusionAreaRankingCriterion::computeRank(const Point2LL& candidate_position) const
{
    return exclusion_area_.inside(candidate_position, true) ? 0 : 1;
}

} // namespace curaengine::types::scoring
