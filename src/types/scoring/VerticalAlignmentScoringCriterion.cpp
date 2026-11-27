// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/VerticalAlignmentScoringCriterion.h"

#include <algorithm>

#include "seam/seam_corners.h"
#include "types/scoring/ElementPosition.h"
#include "utils/math.h"


namespace curaengine::types::scoring
{

using namespace types::geometry;
using namespace utils;

template<typename T>
VerticalAlignmentScoringCriterion<T>::VerticalAlignmentScoringCriterion(const std::vector<Point2LL>& last_layer_seams, const coord_t max_alignment_distance)
    : last_layer_seams_(last_layer_seams)
    , max_alignment_distance_(max_alignment_distance)
{
}

template<typename T>
double VerticalAlignmentScoringCriterion<T>::computeScore(const T& element) const
{
    const Point2LL& candidate_position = getElementPosition(element);
    double max_position_score = 0.0;

    for (const Point2LL& last_layer_seam : last_layer_seams_)
    {
        const double new_score = inverse_lerp(max_alignment_distance_, coord_t{ 0 }, vSize(candidate_position - last_layer_seam));
        max_position_score = std::max(max_position_score, new_score);
    }

    return max_position_score;
}

template class VerticalAlignmentScoringCriterion<seam::SeamCandidate>;

} // namespace curaengine::types::scoring
