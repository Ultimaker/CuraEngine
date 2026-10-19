// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/DistanceScoringCriterion.h"

#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/view/transform.hpp>

#include "geometry/PointsSet.h"


namespace cura
{

DistanceScoringCriterion::DistanceScoringCriterion(const PointsSet& points, const Point2LL& target_position, const DistanceType distance_type, const double distance_divider)
    : DistanceScoringCriterion(points, std::vector<Point2LL>{ target_position }, distance_type, distance_divider)
{
}

DistanceScoringCriterion::DistanceScoringCriterion(
    const PointsSet& points,
    const std::vector<Point2LL>& target_positions,
    const DistanceType distance_type,
    const double distance_divider)
    : PositionBasedScoringCriterion(points)
    , target_positions_(target_positions)
    , distance_type_(distance_type)
    , distance_divider_(distance_divider)
{
}


double DistanceScoringCriterion::computeScore(const Point2LL& candidate_position) const
{
    double score{ 0.0 };

    for (const Point2LL& target_pos : target_positions_)
    {
        double distance = 0.0;
        switch (distance_type_)
        {
        case DistanceType::Euclidian:
            // Use actual (non-squared) distance to ensure a proper scoring distribution
            distance = vSizeMM(candidate_position - target_pos);
            break;
        case DistanceType::XOnly:
            distance = INT2MM(std::abs(candidate_position.X - target_pos.X));
            break;
        case DistanceType::YOnly:
            distance = INT2MM(std::abs(candidate_position.Y - target_pos.Y));
            break;
        }

        // Use reciprocal function to normalize distance score decreasingly
        score = std::max(score, 1.0 / (1.0 + (distance / distance_divider_)));
    }

    return score;
}

} // namespace cura
