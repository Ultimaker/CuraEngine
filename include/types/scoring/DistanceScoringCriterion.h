// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "types/Coord_t.h"
#include "types/geometry/Point2LL.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Criterion that will give a score according to the distance from the position of the element to a target point. Closer elements will get
 * a higher score.
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements that are scored, which must have a position (see getElementPosition())
 */
template<typename T>
class DistanceScoringCriterion : public ScoringCriterion<T>
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
    const std::vector<types::geometry::Point2LL> target_positions_;
    const DistanceType distance_type_;

    /*!
     * Fixed divider for shortest distances computation. The divider should be set so that the minimum encountered
     * distance gives a score very close to 1.0, and a medium-far distance gives a score close to 0.5
     */
    const double distance_divider_;

public:
    explicit DistanceScoringCriterion(
        const types::geometry::Point2LL& target_position,
        const DistanceType distance_type = default_distance_type,
        const double distance_divider = default_distance_divider);

    explicit DistanceScoringCriterion(
        const std::vector<types::geometry::Point2LL>& target_positions,
        const DistanceType distance_type = default_distance_type,
        const double distance_divider = default_distance_divider);

    double computeScore(const T& element) const override;

    /*!
     * Computes the score that an element gets when it is at the given distance of its closest target position. Elements that have a score greater than this value are closer than
     * this distance
     * \param distance The distance to the target position
     * \return The score for this distance
     */
    double computeScoreAtDistance(const types::coord_t distance) const;

private:
    double scoreFromDistance(const double distance_mm) const;
};

} // namespace curaengine::types::scoring
