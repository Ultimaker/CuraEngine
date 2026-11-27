// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/BestElementFinder.h"

#include "types/scoring/CriteriaPass.h"
#include "types/scoring/RankingCriteriaPass.h"
#include "types/scoring/ScoringCriterion.h"
#include "types/scoring/WeighedScoringCriteriaPass.h"

namespace curaengine::types::scoring
{

void BestElementFinder::appendSingleCriterionPass(std::shared_ptr<ScoringCriterion> criterion, const std::optional<double>& outsider_delta_threshold)
{
    auto criteria_pass = std::make_shared<WeighedScoringCriteriaPass>();
    criteria_pass->outsider_delta_threshold = outsider_delta_threshold;
    criteria_pass->appendCriterion(criterion);
    appendCriteriaPass(criteria_pass);
}

void BestElementFinder::appendRankingCriteriaPass(const std::shared_ptr<RankingCriterion>& criterion)
{
    appendCriteriaPass(std::make_shared<RankingCriteriaPass>(criterion));
}

std::optional<size_t> BestElementFinder::findBestElement(const size_t candidates_count)
{
    const std::vector<Candidate> candidates = findBestElements(candidates_count);
    return ! candidates.empty() ? std::make_optional(candidates.front().candidate_index) : std::nullopt;
}

std::vector<Candidate> BestElementFinder::findBestElements(const size_t candidates_count)
{
    // Start by initializing the candidates list in natural order
    std::vector<Candidate> candidates(candidates_count);
    for (size_t i = 0; i < candidates_count; ++i)
    {
        candidates[i].candidate_index = i;
    }

    // Now run the passes until no candidate is left or there are no more passes
    for (const auto& pass : passes_)
    {
        pass->apply(candidates);

        if (candidates.empty())
        {
            break;
        }
    }

    return candidates;
}

} // namespace curaengine::types::scoring
