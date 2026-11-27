// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "seam/seam_corners.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Criterion that will give a score according to the sharpness of the pre-calculated corner of the seam candidate.
 * Depending on the given preference, concave or convex corners may get a higher score.
 */
class CornerScoringCriterion : public ScoringCriterion<seam::SeamCandidate>
{
private:
    double concave_angle_weight_{ 0.0 };
    double convex_angle_weight_{ 0.0 };

public:
    /*!
     * \param corner_preference The kind of corners to favor. The score of the candidates has to be the unweighted one, which is signed (positive is concave, negative is
     * convex)
     */
    explicit CornerScoringCriterion(const types::EZSeamCornerPrefType corner_preference);

    double computeScore(const seam::SeamCandidate& candidate) const override;
};

} // namespace curaengine::types::scoring
