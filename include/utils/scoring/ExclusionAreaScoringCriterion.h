// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/scoring/PositionBasedScoringCriterion.h"

namespace curaengine::geometry
{
class PointsSet;
class Shape;
}

namespace curaengine::utils::scoring
{



/*!
 * Criterion that will give a score according to whether the point is located inside or outside of an exclusion area.
 * This is currently a binary test and the score will be either 0 or 1.
 */
class ExclusionAreaScoringCriterion : public PositionBasedScoringCriterion
{
private:
    const geometry::Shape& exclusion_area_;

public:
    explicit ExclusionAreaScoringCriterion(const geometry::PointsSet& points, const geometry::Shape& exclusion_area);

    virtual double computeScore(const geometry::Point2LL& candidate_position) const override;
};

} // namespace curaengine::utils::scoring
