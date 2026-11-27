// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/Point2LL.h"
#include "types/scoring/RankingCriterion.h"

namespace curaengine::types::geometry
{
class PointsSet;
}

namespace curaengine::types::scoring
{

/*!
 * Abstract criterion that will give a rank according to the position of the point
 */
class PositionBasedRankingCriterion : public RankingCriterion
{
private:
    const types::geometry::PointsSet& points_;

public:
    explicit PositionBasedRankingCriterion(const types::geometry::PointsSet& points);

    uint8_t computeRank(const size_t candidate_index) const override final;

    virtual uint8_t computeRank(const types::geometry::Point2LL& candidate_position) const = 0;

protected:
    const types::geometry::PointsSet& getPoints() const
    {
        return points_;
    }
};

} // namespace curaengine::types::scoring
