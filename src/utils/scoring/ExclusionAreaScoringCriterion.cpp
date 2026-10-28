// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/ExclusionAreaScoringCriterion.h"

#include "geometry/Shape.h"


namespace curaengine::utils::scoring
{

ExclusionAreaScoringCriterion::ExclusionAreaScoringCriterion(const ::curaengine::geometry::PointsSet& points, const ::curaengine::geometry::Shape& exclusion_area)
    : PositionBasedScoringCriterion(points)
    , exclusion_area_(exclusion_area)
{
}

double ExclusionAreaScoringCriterion::computeScore(const ::curaengine::geometry::Point2LL& candidate_position) const
{
    return exclusion_area_.inside(candidate_position, true) ? 0.0 : 1.0;
}

} // namespace curaengine::utils::scoring
