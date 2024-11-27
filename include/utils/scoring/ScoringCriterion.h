// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SCORING_SCORINGCRITERION_H
#define UTILS_SCORING_SCORINGCRITERION_H

#include <stddef.h>

namespace cura
{

/*!
 * Base class for implementing a selection criterion when calculating a multi-criteria score to select the best element
 * amongst a list.
 */
class ScoringCriterion
{
public:
    ScoringCriterion() = default;

    virtual ~ScoringCriterion() = default;

    /*!
     * \brief Computes the score of an element regarding this criterion. To ensure a proper selection, this value must
     *        be contained in [0.0, 1.0] and the different given scores must be evenly distributed in this range.
     * \param candidate_index The index of the candidate of the original list
     * \return The raw score of the element regarding this criterion
     */
    virtual double computeScore(const size_t candidate_index) const = 0;
};

} // namespace cura

#endif // UTILS_SCORING_SCORINGCRITERION_H
