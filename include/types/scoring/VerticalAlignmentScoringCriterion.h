// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "types/Coord_t.h"
#include "types/geometry/Point2LL.h"
#include "types/scoring/PositionBasedScoringCriterion.h"

namespace curaengine::types::scoring
{
class VerticalAlignmentScoringCriterion : public PositionBasedScoringCriterion
{
public:
    explicit VerticalAlignmentScoringCriterion(
        const types::geometry::PointsSet& points,
        const std::vector<types::geometry::Point2LL>& last_layer_seams,
        const types::coord_t max_alignment_distance);

    double computeScore(const types::geometry::Point2LL& candidate_position) const override;

private:
    std::vector<types::geometry::Point2LL> last_layer_seams_;
    const types::coord_t max_alignment_distance_;
};

} // namespace curaengine::types::scoring
