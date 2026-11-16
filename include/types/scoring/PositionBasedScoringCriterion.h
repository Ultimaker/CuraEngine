// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/Point2LL.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::geometry
{
class PointsSet;
}

namespace curaengine::types::scoring
{


/*!
 * Abstract criterion that will give a score according to the position of the point
 */
class PositionBasedScoringCriterion : public ScoringCriterion
{
private:
    const types::geometry::PointsSet& points_;

public:
    explicit PositionBasedScoringCriterion(const types::geometry::PointsSet& points);

    virtual double computeScore(const size_t candidate_index) const override;

    virtual double computeScore(const types::geometry::Point2LL& candidate_position) const;

protected:
    const types::geometry::PointsSet& getPoints() const
    {
        return points_;
    }
};

} // namespace curaengine::types::scoring
