// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <vector>

#include "seam/seam_corners.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Criterion that will give a score according to the sharpness of the corners that have been pre-calculated on the path.
 * Depending on the given preference, concave or convex corners may get a higher score.
 */
class CornerScoringCriterion : public ScoringCriterion
{
public:
    /*!
     * \param corners The corners to be scored. Each candidate is the corner of the same index, and its score has to be the unweighted one,
     * which is signed (positive is concave, negative is convex)
     * \param corner_preference The kind of corners to favor
     */
    explicit CornerScoringCriterion(const std::vector<seam::SeamCandidate>& corners, const types::EZSeamCornerPrefType corner_preference);

    double computeScore(const size_t candidate_index) const override;

private:
    const std::vector<seam::SeamCandidate>& corners_;
    double concave_angle_weight_{ 0.0 };
    double convex_angle_weight_{ 0.0 };
};

} // namespace curaengine::types::scoring
