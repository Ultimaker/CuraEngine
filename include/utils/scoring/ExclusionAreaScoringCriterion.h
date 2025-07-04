// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXCLUSIONAREASCORINGCRITERION_H
#define EXCLUSIONAREASCORINGCRITERION_H

#include <stddef.h>

#include "utils/scoring/PositionBasedScoringCriterion.h"

namespace cura
{
class PointsSet;
class Shape;

/*!
 * Criterion that will give a score according to whether the point is located inside or outside of an exclusion area.
 * This is currently a binary test and the score will be either 0 or 1.
 */
class ExclusionAreaScoringCriterion : public PositionBasedScoringCriterion
{
private:
    const Shape& exclusion_area_;

public:
    explicit ExclusionAreaScoringCriterion(const PointsSet& points, const Shape& exclusion_area);

    virtual double computeScore(const Point2LL& candidate_position) const override;
};

} // namespace cura

#endif // EXCLUSIONAREASCORINGCRITERION_H
