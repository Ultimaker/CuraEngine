// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXCLUSIONAREASCORINGCRITERION_H
#define EXCLUSIONAREASCORINGCRITERION_H

#include <memory>
#include <vector>

#include "utils/scoring/ScoringCriterion.h"

namespace cura
{
class Shape;
struct StartCandidatePoint;

/*!
 * Criterion that will give a score according to whether the point is located inside or outside of an exclusion area.
 * This is currently a binary test and the score will be either 0 or 1.
 */
class ExclusionAreaScoringCriterion : public ScoringCriterion
{
private:
    const std::vector<StartCandidatePoint>& points_;
    const std::shared_ptr<Shape> exclusion_area_;

public:
    explicit ExclusionAreaScoringCriterion(const std::vector<StartCandidatePoint>& points, const std::shared_ptr<Shape>& exclusion_area);

    double computeScore(const size_t candidate_index) const override;
};

} // namespace cura

#endif // EXCLUSIONAREASCORINGCRITERION_H
