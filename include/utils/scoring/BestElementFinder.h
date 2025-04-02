// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SCORING_BESTCANDIDATEFINDER_H
#define UTILS_SCORING_BESTCANDIDATEFINDER_H

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace cura
{
class ScoringCriterion;

/*!
 * This class implements an algorithm to find an element amongst a list, regarding one or multiple scoring criteria. The
 * criteria are implemented by subclassing the CriterionScoring class. It is also possible to set up multiple passes of
 * criteria. Thus, if the first pass gives a few best candidates that are too close to each other, we keep only the best
 * candidates and process a second pass with different criteria, and so on until we have a single outsider, or no more
 * criteria.
 */
class BestElementFinder
{
private:
    /*!
     * Contains the index of an element in the source list, and its calculated score
     */
    struct Candidate
    {
        size_t candidate_index;
        double score{ 0.0 };
    };

public:
    /*!
     * Contains a criterion to be processed to calculate the score of an element, and the weight is has on the global
     * score calculation.
     */
    struct WeighedCriterion
    {
        std::shared_ptr<ScoringCriterion> criterion;

        /*!
         * The weight to be given when taking this criterion into the global score. A score that contributes "normally"
         * to the global score should have a weight of 1.0, and others should be adjusted around this value, to give
         * them more or less influence.
         */
        double weight{ 1.0 };
    };

    /*!
     * Contains multiple criteria to be processed on each element in a single pass
     */
    struct CriteriaPass
    {
        std::vector<WeighedCriterion> criteria;

        /*!
         * Once we have calculated the global scores of each element for this pass, we calculate the score difference
         * between the best candidate and the following ones. If the following ones have a score close enough to the
         * best, within this threshold, they will also be considered outsiders and will be run for the next pass.
         * This value will be ignored for the last pass.
         */
        double outsider_delta_threshold{ 0.0 };
    };

private:
    std::vector<CriteriaPass> criteria_;

public:
    explicit BestElementFinder() = default;

    void appendCriteriaPass(const CriteriaPass& pass)
    {
        criteria_.push_back(pass);
    }

    /*!
     * Convenience method to add a pass with a single criterion
     */
    void appendSingleCriterionPass(std::shared_ptr<ScoringCriterion> criterion, const double outsider_delta_threshold = 0.0);

    std::optional<size_t> findBestElement(const size_t candidates_count);
};

} // namespace cura
#endif // UTILS_SCORING_BESTCANDIDATEFINDER_H
