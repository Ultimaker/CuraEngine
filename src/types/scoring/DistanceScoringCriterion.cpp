// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/DistanceScoringCriterion.h"

#include <algorithm>

#include "seam/seam_corners.h"
#include "types/scoring/ElementPosition.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

template<typename T>
DistanceScoringCriterion<T>::DistanceScoringCriterion(const Point2LL& target_position, const DistanceType distance_type, const double distance_divider)
    : DistanceScoringCriterion(std::vector<Point2LL>{ target_position }, distance_type, distance_divider)
{
}

template<typename T>
DistanceScoringCriterion<T>::DistanceScoringCriterion(const std::vector<Point2LL>& target_positions, const DistanceType distance_type, const double distance_divider)
    : target_positions_(target_positions)
    , distance_type_(distance_type)
    , distance_divider_(distance_divider)
{
}

template<typename T>
double DistanceScoringCriterion<T>::computeScore(const T& element) const
{
    const Point2LL& candidate_position = getElementPosition(element);
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
            distance = intToMm(std::abs(candidate_position.X - target_pos.X));
            break;
        case DistanceType::YOnly:
            distance = intToMm(std::abs(candidate_position.Y - target_pos.Y));
            break;
        }

        // Use reciprocal function to normalize distance score decreasingly
        score = std::max(score, 1.0 / (1.0 + (distance / distance_divider_)));
    }

    return score;
}

template class DistanceScoringCriterion<seam::SeamCandidate>;
template class DistanceScoringCriterion<Point2LL>;

} // namespace curaengine::types::scoring
