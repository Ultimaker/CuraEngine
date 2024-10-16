// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_CRITERION_SCORE_H
#define UTILS_CRITERION_SCORE_H

#include <cassert>

#include "SparsePointGrid.h"

namespace cura
{

/*!
 * This structure represents a score given by a single crtierion when calculating a global score to select a best
 * candidate among a list with multiple criteria.
 */
struct CriterionScore
{
    /*!
     * The score given by the criterion. To ensure a proper selection, this value must be contained in [0.0, 1.0] and
     * the different given scores must be evenly distributed in this range.
     */
    double score{ 0.0 };

    /*!
     * The weight to be given when taking this score into the global score. A score that contributes "normally" to the
     * global score should have a weight of 1.0, and others should be adjusted around this value, to give them more or
     * less influence.
     */
    double weight{ 0.0 };
};

} // namespace cura
#endif // UTILS_CRITERION_SCORE_H
