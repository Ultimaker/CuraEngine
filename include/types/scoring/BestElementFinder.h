// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "types/scoring/CriteriaPass.h"

namespace curaengine::types::scoring
{
class RankingCriterion;
class ScoringCriterion;

/*!
 * This class implements an algorithm to find an element amongst a list, regarding one or multiple selection passes. The
 * passes are implemented by subclassing the CriteriaPass class, for example with scoring criteria (see
 * WeighedScoringCriteriaPass) or ranking criteria (see RankingCriteriaPass). Thus, if the first pass gives a few best
 * candidates that are too close to each other, we keep only the best candidates and process a second pass with different
 * criteria, and so on until we have a single outsider, or no more passes.
 */
class BestElementFinder
{
private:
    std::vector<std::shared_ptr<CriteriaPass>> passes_;

public:
    explicit BestElementFinder() = default;

    void appendCriteriaPass(const std::shared_ptr<CriteriaPass>& pass)
    {
        passes_.push_back(pass);
    }

    /*! Convenience method to add a weighed scoring pass with a single criterion */
    void appendSingleCriterionPass(std::shared_ptr<ScoringCriterion> criterion, const std::optional<double>& outsider_delta_threshold = 0.0);

    /*!
     * Convenience method to add a ranking pass with a single criterion
     */
    void appendRankingCriteriaPass(const std::shared_ptr<RankingCriterion>& criterion);

    /*!
     * Runs all the passes on the candidates, and returns the best one
     * \param candidates_count The number of candidates in the original list
     * \return The index of the best candidate in the original list, or nullopt if no candidate remains
     */
    [[nodiscard]] std::optional<size_t> findBestElement(const size_t candidates_count);

    /*!
     * Runs all the passes on the candidates, and returns all the ones that remain
     * \param candidates_count The number of candidates in the original list
     * \return The remaining candidates, from the best one to the worst one. The score of each one is the one that was
     * calculated by the last pass that calculates scores, which may not be significant if there is none
     */
    [[nodiscard]] std::vector<Candidate> findBestElements(const size_t candidates_count);
};

} // namespace curaengine::types::scoring
