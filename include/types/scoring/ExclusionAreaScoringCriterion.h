// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/scoring/PositionBasedScoringCriterion.h"

namespace curaengine::types::geometry
{
class PointsSet;
class Shape;
} // namespace curaengine::types::geometry

namespace curaengine::types::scoring
{


/*!
 * Criterion that will give a score according to whether the point is located inside or outside of an exclusion area.
 * This is currently a binary test and the score will be either 0 or 1.
 */
class ExclusionAreaScoringCriterion : public PositionBasedScoringCriterion
{
private:
    const types::geometry::Shape& exclusion_area_;

public:
    explicit ExclusionAreaScoringCriterion(const types::geometry::PointsSet& points, const types::geometry::Shape& exclusion_area);

    virtual double computeScore(const types::geometry::Point2LL& candidate_position) const override;
};

} // namespace curaengine::types::scoring
