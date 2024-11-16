// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/DistanceScoringCriterion.h"

#include "geometry/PointsSet.h"
#include "path_processing/StartCandidatePoint.h"


namespace cura
{

DistanceScoringCriterion::DistanceScoringCriterion(const std::vector<StartCandidatePoint>& points, const Point3LL& target_pos, DistanceType distance_type)
    : points_(points)
    , target_pos_(target_pos)
    , distance_type_(distance_type)
{
}

double DistanceScoringCriterion::computeScore(const size_t candidate_index) const
{
    const Point2LL candidate_position = points_.at(candidate_index).position.toPoint2LL();

    // Fixed divider for shortest distances computation. The divider should be set so that the minimum encountered
    // distance gives a score very close to 1.0, and a medium-far distance gives a score close to 0.5
    constexpr double distance_divider = 20.0;

    double distance = 0.0;
    switch (distance_type_)
    {
    case DistanceType::Euclidian:
        // Use actual (non-squared) distance to ensure a proper scoring distribution
        distance = vSizeMM(candidate_position - target_pos_);
        break;
    case DistanceType::XOnly:
        distance = INT2MM(std::abs(candidate_position.X - target_pos_.x_));
        break;
    case DistanceType::YOnly:
        distance = INT2MM(std::abs(candidate_position.Y - target_pos_.y_));
        break;
    }

    // Use reciprocal function to normalize distance score decreasingly
    return 1.0 / (1.0 + (distance / distance_divider));
}

} // namespace cura
