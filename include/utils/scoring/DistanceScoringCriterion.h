// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DISTANCESCORINGCRITERION_H
#define DISTANCESCORINGCRITERION_H

#include <vector>

#include "utils/scoring/ScoringCriterion.h"

namespace cura
{
class PointsSet;
class Point3LL;
struct StartCandidatePoint;

/*!
 * Criterion that will give a score according to the distance from the point to a target point. Closer points will get
 * a higher score.
 */
class DistanceScoringCriterion : public ScoringCriterion
{
public:
    enum class DistanceType
    {
        Euclidian, // Classic euclidian distance between the points
        XOnly, // Only difference on X coordinate
        YOnly, // Only difference on Y coordinate
    };

private:
    const std::vector<StartCandidatePoint>& points_;
    const Point3LL& target_pos_;
    const DistanceType distance_type_;

public:
    explicit DistanceScoringCriterion(const std::vector<StartCandidatePoint>& points, const Point3LL& target_pos, DistanceType distance_type = DistanceType::Euclidian);

    double computeScore(const size_t candidate_index) const override;
};

} // namespace cura

#endif // DISTANCESCORINGCRITERION_H
