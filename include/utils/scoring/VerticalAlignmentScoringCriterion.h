// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"
#include "utils/scoring/PositionBasedScoringCriterion.h"

namespace cura
{
class VerticalAlignmentScoringCriterion : public PositionBasedScoringCriterion
{
public:
    explicit VerticalAlignmentScoringCriterion(const PointsSet& points, const std::vector<Point2LL>& last_layer_seams, const coord_t max_alignment_distance);

    double computeScore(const Point2LL& candidate_position) const override;

private:
    std::vector<Point2LL> last_layer_seams_;
    const coord_t max_alignment_distance_;
};

} // namespace cura
