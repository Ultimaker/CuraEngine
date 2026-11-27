// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/scoring/PositionBasedRankingCriterion.h"

namespace curaengine::types::geometry
{
class PointsSet;
class Shape;
} // namespace curaengine::types::geometry

namespace curaengine::types::scoring
{


/*!
 * Criterion that will give a rank according to whether the point is located inside or outside of an exclusion area.
 * This is currently a binary test and the rank will be either 0 (inside) or 1 (outside).
 */
class ExclusionAreaRankingCriterion : public PositionBasedRankingCriterion
{
private:
    const types::geometry::Shape& exclusion_area_;

public:
    explicit ExclusionAreaRankingCriterion(const types::geometry::PointsSet& points, const types::geometry::Shape& exclusion_area);

    virtual uint8_t computeRank(const types::geometry::Point2LL& candidate_position) const override;
};

} // namespace curaengine::types::scoring
