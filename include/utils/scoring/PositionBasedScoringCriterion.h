// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POSITIONBASEDSCORINGCRITERION_H
#define POSITIONBASEDSCORINGCRITERION_H

#include "geometry/Point2LL.h"
#include "utils/scoring/ScoringCriterion.h"

namespace cura
{
class PointsSet;

/*!
 * Abstract criterion that will give a score according to the position of the point
 */
class PositionBasedScoringCriterion : public ScoringCriterion
{
private:
    const PointsSet& points_;

public:
    explicit PositionBasedScoringCriterion(const PointsSet& points);

    virtual double computeScore(const size_t candidate_index) const override;

    virtual double computeScore(const Point2LL& candidate_position) const;

protected:
    const PointsSet& getPoints() const
    {
        return points_;
    }
};

} // namespace cura

#endif // POSITIONBASEDSCORINGCRITERION_H
