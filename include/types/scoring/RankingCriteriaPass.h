// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>

#include "types/scoring/CriteriaPass.h"

namespace curaengine::types::scoring
{
class RankingCriterion;

/*!
 * Pass that ranks the candidates with a ranking criterion, and keeps only the ones that have the highest rank
 */
class RankingCriteriaPass : public CriteriaPass
{
public:
    explicit RankingCriteriaPass(const std::shared_ptr<RankingCriterion>& criterion);

    void apply(std::vector<Candidate>& candidates) const override;

private:
    std::shared_ptr<RankingCriterion> criterion_;
};

} // namespace curaengine::types::scoring
