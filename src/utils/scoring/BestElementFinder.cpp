// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/BestElementFinder.h"

#include <limits>

#include <range/v3/view/enumerate.hpp>

#include "utils/scoring/ScoringCriterion.h"

namespace cura
{

void BestElementFinder::appendSingleCriterionPass(std::shared_ptr<ScoringCriterion> criterion, const double outsider_delta_threshold)
{
    WeighedCriterion weighed_criterion;
    weighed_criterion.criterion = criterion;

    CriteriaPass criteria_pass;
    criteria_pass.outsider_delta_threshold = outsider_delta_threshold;
    criteria_pass.criteria.push_back(weighed_criterion);
    appendCriteriaPass(criteria_pass);
}

std::optional<size_t> cura::BestElementFinder::findBestElement(const size_t candidates_count)
{
    // Start by initializing the candidates list in natural order
    std::vector<Candidate> best_candidates(candidates_count);
    for (size_t i = 0; i < candidates_count; ++i)
    {
        best_candidates[i].candidate_index = i;
    }

    const auto begin = best_candidates.begin();
    auto end = best_candidates.end();

    // Now run the criteria passes until we have a single outsider or no more criteria
    for (const auto& [pass_index, criteria_pass] : criteria_ | ranges::views::enumerate)
    {
        // For each element, reset score, process each criterion and apply weights to get the global score
        auto best_candidate_iterator = end;
        for (auto iterator = begin; iterator != end; ++iterator)
        {
            iterator->score = 0.0;

            for (const auto& weighed_criterion : criteria_pass.criteria)
            {
                iterator->score += weighed_criterion.criterion->computeScore(iterator->candidate_index) * weighed_criterion.weight;
            }

            if (best_candidate_iterator == end || iterator->score > best_candidate_iterator->score)
            {
                best_candidate_iterator = iterator;
            }
        }

        if (best_candidate_iterator == end)
        {
            // Something went wrong, we don't have a best candidate
            return std::nullopt;
        }

        // Early out for last pass, just keep the best candidate
        if (pass_index == criteria_.size() - 1)
        {
            return best_candidate_iterator->candidate_index;
        }

        // Skip candidates that have a score too far from the actual best one
        const double delta_threshold = criteria_pass.outsider_delta_threshold + std::numeric_limits<double>::epsilon();
        end = std::remove_if(
            begin,
            end,
            [&best_candidate_iterator, &delta_threshold](const Candidate& candidate)
            {
                return best_candidate_iterator->score - candidate.score > delta_threshold;
            });

        if (std::distance(begin, end) == 1)
        {
            // We have a single outsider, don't go further
            return begin->candidate_index;
        }
    }

    return begin != end ? std::make_optional(begin->candidate_index) : std::nullopt;
}

} // namespace cura
