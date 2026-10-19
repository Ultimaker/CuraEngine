// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DISTANCESCORINGCRITERION_H
#define DISTANCESCORINGCRITERION_H

#include "utils/scoring/PositionBasedScoringCriterion.h"

namespace cura
{
class PointsSet;

/*!
 * Criterion that will give a score according to the distance from the point to a target point. Closer points will get
 * a higher score.
 */
class DistanceScoringCriterion : public PositionBasedScoringCriterion
{
public:
    enum class DistanceType
    {
        Euclidian, // Classic euclidian distance between the points
        XOnly, // Only difference on X coordinate
        YOnly, // Only difference on Y coordinate
    };

    static constexpr DistanceType default_distance_type{ DistanceType::Euclidian };
    static constexpr double default_distance_divider{ 20.0 };

private:
    const std::vector<Point2LL> target_positions_;
    const DistanceType distance_type_;

    /*!
     * Fixed divider for shortest distances computation. The divider should be set so that the minimum encountered
     * distance gives a score very close to 1.0, and a medium-far distance gives a score close to 0.5
     */
    const double distance_divider_;

public:
    explicit DistanceScoringCriterion(
        const PointsSet& points,
        const Point2LL& target_position,
        const DistanceType distance_type = default_distance_type,
        const double distance_divider = default_distance_divider);

    explicit DistanceScoringCriterion(
        const PointsSet& points,
        const std::vector<Point2LL>& target_positions,
        const DistanceType distance_type = default_distance_type,
        const double distance_divider = default_distance_divider);

    virtual double computeScore(const Point2LL& candidate_position) const override;
};

} // namespace cura

#endif // DISTANCESCORINGCRITERION_H
