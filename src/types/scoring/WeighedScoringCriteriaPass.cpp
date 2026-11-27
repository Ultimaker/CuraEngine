// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/WeighedScoringCriteriaPass.h"

#include <algorithm>
#include <cassert>
#include <limits>

#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

void WeighedScoringCriteriaPass::appendCriterion(const std::shared_ptr<ScoringCriterion>& criterion, const double weight)
{
    criteria.emplace_back(criterion, weight);
}

void WeighedScoringCriteriaPass::apply(std::vector<Candidate>& candidates) const
{
    // For each element, reset score, process each criterion and apply weights to get the global score
    auto best_candidate_iterator = candidates.end();
    for (auto iterator = candidates.begin(); iterator != candidates.end(); ++iterator)
    {
        iterator->score = 0.0;

        for (const auto& weighed_criterion : criteria)
        {
            const double criterion_score = weighed_criterion.criterion->computeScore(iterator->candidate_index);
            assert(criterion_score >= 0.0 && criterion_score <= 1.0 && "Criterion score is out-of-bounds");
            iterator->score += criterion_score * weighed_criterion.weight;
        }

        if (best_candidate_iterator == candidates.end() || iterator->score > best_candidate_iterator->score)
        {
            best_candidate_iterator = iterator;
        }
    }

    if (best_candidate_iterator == candidates.end())
    {
        return;
    }

    const double best_score = best_candidate_iterator->score;

    if (min_score.has_value())
    {
        std::erase_if(
            candidates,
            [this](const Candidate& candidate)
            {
                return ! (candidate.score > min_score.value());
            });
    }

    // Skip candidates that have a score too far from the actual best one
    if (outsider_delta_threshold.has_value())
    {
        const double delta_threshold = outsider_delta_threshold.value() + std::numeric_limits<double>::epsilon();
        std::erase_if(
            candidates,
            [&best_score, &delta_threshold](const Candidate& candidate)
            {
                return best_score - candidate.score > delta_threshold;
            });
    }

    // Sort the remaining candidates by score, so that if the next pass doesn't differentiate them, we keep the best one from this pass
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& candidate1, const Candidate& candidate2)
        {
            return candidate1.score > candidate2.score;
        });
}

} // namespace curaengine::types::scoring
