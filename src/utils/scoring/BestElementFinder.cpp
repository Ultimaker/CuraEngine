// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/BestElementFinder.h"

#include "utils/scoring/ScoringCriterion.h"

namespace cura
{

std::optional<size_t> cura::BestElementFinder::findBestElement(const size_t candidates_count)
{
    if (candidates_count == 0)
    {
        return std::nullopt;
    }

    // Start by initializing the candidates list in natural order
    std::vector<Candidate> candidates(candidates_count);
    for (size_t i = 0; i < candidates_count; ++i)
    {
        candidates[i].candidate_index = i;
    }

    const auto begin = candidates.begin();
    auto end = candidates.end();

    // Now run the criteria passes until we have a single outsider or no more cirteria
    for (const CriteriaPass& criteria_pass : criteria_)
    {
        // For each element, reset score, process each criterion and apply wweights to get the global score
        for (auto iterator = begin; iterator != end; ++iterator)
        {
            iterator->score = 0.0;

            for (const auto& weighed_criterion : criteria_pass.criteria)
            {
                iterator->score += weighed_criterion.criterion->computeScore(iterator->candidate_index) * weighed_criterion.weight;
            }
        }

        // Now sort the candiates (sub)list to get the best candidates first
        std::sort(
            begin,
            end,
            [](const Candidate& candidate1, const Candidate& candidate2)
            {
                return candidate1.score > candidate2.score;
            });

        // Check whether the following candidates have a score similar enough to the actual best one, and skip others
        if (criteria_pass.outsider_delta_threshold > 0.0)
        {
            const double highest_score = candidates.front().score;
            auto iterator = std::next(begin);
            while (iterator != end && (highest_score - iterator->score) <= criteria_pass.outsider_delta_threshold)
            {
                ++iterator;
            }

            end = iterator;
        }

        if (std::distance(begin, end) <= 1)
        {
            // We have a single outsider, don't go further
            break;
        }
    }

    return begin->candidate_index;
}

} // namespace cura
