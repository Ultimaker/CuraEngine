// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/ExclusionAreaScoringCriterion.h"

#include "types/geometry/Shape.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

ExclusionAreaScoringCriterion::ExclusionAreaScoringCriterion(const PointsSet& points, const Shape& exclusion_area)
    : PositionBasedScoringCriterion(points)
    , exclusion_area_(exclusion_area)
{
}

double ExclusionAreaScoringCriterion::computeScore(const Point2LL& candidate_position) const
{
    return exclusion_area_.inside(candidate_position, true) ? 0.0 : 1.0;
}

} // namespace curaengine::types::scoring
