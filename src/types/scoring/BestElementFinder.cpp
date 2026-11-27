// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/BestElementFinder.h"

#include "seam/seam_corners.h"
#include "types/AngleDegrees.h"
#include "types/geometry/Point2LL.h"
#include "types/scoring/CriteriaPass.h"
#include "types/scoring/ScoringCriterion.h"
#include "types/scoring/WeighedScoringCriteriaPass.h"

namespace curaengine::types::scoring
{

template<typename T>
void BestElementFinder<T>::appendCriteriaPass(const std::shared_ptr<CriteriaPass<T>>& pass)
{
    passes_.push_back(pass);
}

template<typename T>
bool BestElementFinder<T>::isEmpty() const
{
    return passes_.empty();
}

template<typename T>
void BestElementFinder<T>::appendSingleCriterionPass(std::shared_ptr<ScoringCriterion<T>> criterion, const std::optional<double>& outsider_delta_threshold)
{
    auto criteria_pass = std::make_shared<WeighedScoringCriteriaPass<T>>();
    criteria_pass->outsider_delta_threshold = outsider_delta_threshold;
    criteria_pass->appendCriterion(criterion);
    appendCriteriaPass(criteria_pass);
}

template<typename T>
std::optional<Candidate<T>> BestElementFinder<T>::findBestElement(const std::span<const T> elements) const
{
    const std::vector<Candidate<T>> candidates = findBestElements(elements);
    return ! candidates.empty() ? std::make_optional(candidates.front()) : std::nullopt;
}

template<typename T>
std::vector<Candidate<T>> BestElementFinder<T>::findBestElements(const std::span<const T> elements) const
{
    // Start by initializing the candidates list in natural order
    std::vector<Candidate<T>> candidates;
    candidates.reserve(elements.size());
    for (size_t i = 0; i < elements.size(); ++i)
    {
        candidates.push_back(Candidate<T>{ .element = &elements[i], .candidate_index = i });
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

template class BestElementFinder<seam::SeamCandidate>;
template class BestElementFinder<geometry::Point2LL>;
template class BestElementFinder<AngleDegrees>;

} // namespace curaengine::types::scoring
