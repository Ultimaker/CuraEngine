// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/DistanceScoringCriterion.h"

#include "geometry/PointsSet.h"


namespace cura
{

DistanceScoringCriterion::DistanceScoringCriterion(const PointsSet& points, const Point2LL& target_pos, DistanceType distance_type, const double distance_divider)
    : points_(points)
    , target_pos_(target_pos)
    , distance_type_(distance_type)
    , distance_divider_(distance_divider)
{
}

double DistanceScoringCriterion::computeScore(const size_t candidate_index) const
{
    const Point2LL& candidate_position = points_.at(candidate_index);

    double distance = 0.0;
    switch (distance_type_)
    {
    case DistanceType::Euclidian:
        // Use actual (non-squared) distance to ensure a proper scoring distribution
        distance = vSizeMM(candidate_position - target_pos_);
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

} // namespace cura
