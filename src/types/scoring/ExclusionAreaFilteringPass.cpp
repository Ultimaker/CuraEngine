// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/ExclusionAreaFilteringPass.h"

#include <range/v3/algorithm/partition.hpp>

#include "seam/seam_corners.h"
#include "types/geometry/Shape.h"
#include "types/scoring/ElementPosition.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;

template<typename T>
ExclusionAreaFilteringPass<T>::ExclusionAreaFilteringPass(const Shape& exclusion_area)
    : exclusion_area_(exclusion_area)
{
}

template<typename T>
ExclusionAreaFilteringPass<T>::ExclusionAreaFilteringPass(Shape&& exclusion_area)
    : exclusion_area_(std::move(exclusion_area))
{
}

template<typename T>
void ExclusionAreaFilteringPass<T>::filter(std::vector<Candidate<T>>& candidates) const
{
    const auto pivot = ranges::partition(
        candidates,
        [this](const Candidate<T>& candidate)
        {
            return ! exclusion_area_.inside(getElementPosition(*candidate.element), true);
        });

    if (pivot != candidates.begin())
    {
        // If there is at least one non-excluded candidate, remove the others
        candidates.erase(pivot, candidates.end());
    }
}

template class ExclusionAreaFilteringPass<seam::SeamCandidate>;
template class ExclusionAreaFilteringPass<Point2LL>;

} // namespace curaengine::types::scoring
