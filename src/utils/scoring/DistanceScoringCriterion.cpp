// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/DistanceScoringCriterion.h"

#include "geometry/PointsSet.h"


namespace curaengine::utils::scoring
{

DistanceScoringCriterion::DistanceScoringCriterion(const ::curaengine::geometry::PointsSet& points, const ::curaengine::geometry::Point2LL& target_pos, DistanceType distance_type, const double distance_divider)
    : PositionBasedScoringCriterion(points)
    , target_pos_(target_pos)
    , distance_type_(distance_type)
    , distance_divider_(distance_divider)
{
}

double DistanceScoringCriterion::computeScore(const ::curaengine::geometry::Point2LL& candidate_position) const
{
    double distance = 0.0;
    switch (distance_type_)
    {
    case DistanceType::Euclidian:
        // Use actual (non-squared) distance to ensure a proper scoring distribution
        distance = ::curaengine::geometry::vSizeMM(candidate_position - target_pos_);
        break;
    case DistanceType::XOnly:
        distance = INT2MM(std::abs(candidate_position.X - target_pos_.X));
        break;
    case DistanceType::YOnly:
        distance = INT2MM(std::abs(candidate_position.Y - target_pos_.Y));
        break;
    }

    // Use reciprocal function to normalize distance score decreasingly
    return 1.0 / (1.0 + (distance / distance_divider_));
}

} // namespace curaengine::utils::scoring
