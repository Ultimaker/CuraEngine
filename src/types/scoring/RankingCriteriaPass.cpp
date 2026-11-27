// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/RankingCriteriaPass.h"

#include <algorithm>

#include "types/scoring/RankingCriterion.h"

namespace curaengine::types::scoring
{

RankingCriteriaPass::RankingCriteriaPass(const std::shared_ptr<RankingCriterion>& criterion)
    : criterion_(criterion)
{
}

void RankingCriteriaPass::apply(std::vector<Candidate>& candidates) const
{
    if (candidates.empty())
    {
        return;
    }

    // There are only a few ranks, so a single pass that finds the highest one is enough
    std::vector<uint8_t> ranks;
    ranks.reserve(candidates.size());
    uint8_t highest_rank = 0;
    for (const Candidate& candidate : candidates)
    {
        const uint8_t rank = criterion_->computeRank(candidate.candidate_index);
        ranks.push_back(rank);
        highest_rank = std::max(highest_rank, rank);
    }

    // Keep only the candidates of the highest rank, in the same order
    size_t kept_count = 0;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (ranks[i] == highest_rank)
        {
            candidates[kept_count++] = candidates[i];
        }
    }
    candidates.erase(candidates.begin() + static_cast<std::ptrdiff_t>(kept_count), candidates.end());
}

} // namespace curaengine::types::scoring
