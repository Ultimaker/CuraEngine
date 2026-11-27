// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "types/scoring/CriteriaPass.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Pass that calculates a global score for each candidate, by processing multiple scoring criteria that each have a weight.
 * \tparam T The type of the elements that are processed
 */
template<typename T>
class WeighedScoringCriteriaPass : public CriteriaPass<T>
{
public:
    /*!
     * Contains a criterion to be processed to calculate the score of an element, and the weight is has on the global
     * score calculation.
     */
    struct WeighedCriterion
    {
        std::shared_ptr<ScoringCriterion<T>> criterion;

        /*!
         * The weight to be given when taking this criterion into the global score. A score that contributes "normally"
         * to the global score should have a weight of 1.0, and others should be adjusted around this value, to give
         * them more or less influence.
         */
        double weight{ 1.0 };
    };

    std::vector<WeighedCriterion> criteria;

    /*!
     * Once we have calculated the global scores of each element for this pass, we calculate the score difference
     * between the best candidate and the following ones. If the following ones have a score close enough to the
     * best, within this threshold, they will also be considered outsiders and will be run for the next pass.
     * If not set, no candidate is removed because of its distance to the best one, the pass only sets the scores and
     * sorts the candidates.
     */
    std::optional<double> outsider_delta_threshold;

    /*!
     * If set, only the candidates that have a score strictly greater than this value are kept
     */
    std::optional<double> min_score;

    /*!
     * Convenience method to add a criterion
     */
    void appendCriterion(const std::shared_ptr<ScoringCriterion<T>>& criterion, const double weight = 1.0);

    void apply(std::vector<Candidate<T>>& candidates) const override;
};

} // namespace curaengine::types::scoring
