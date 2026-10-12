// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/VerticalAlignmentScoringCriterion.h"

#include <spdlog/spdlog.h>

#include "geometry/PointsSet.h"


namespace cura
{

VerticalAlignmentScoringCriterion::VerticalAlignmentScoringCriterion(const PointsSet& points, const std::vector<Point2LL>& last_layer_seams, const coord_t max_alignment_distance)
    : PositionBasedScoringCriterion(points)
    , last_layer_seams_(last_layer_seams)
    , max_alignment_distance_(max_alignment_distance)
{
}

double VerticalAlignmentScoringCriterion::computeScore(const Point2LL& candidate_position) const
{
    double max_position_score = 0.0;

    for (const Point2LL& last_layer_seam : last_layer_seams_)
    {
        const double new_score = inverse_lerp(max_alignment_distance_, coord_t{ 0 }, vSize(candidate_position - last_layer_seam));
        max_position_score = std::max(max_position_score, new_score);
    }

    return max_position_score;
}

} // namespace cura
