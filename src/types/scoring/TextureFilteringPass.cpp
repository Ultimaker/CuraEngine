// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/TextureFilteringPass.h"

#include <algorithm>

#include "TextureArea.h"
#include "TextureDataProvider.h"
#include "seam/seam_corners.h"
#include "types/scoring/ElementPosition.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;

template<typename T>
TextureFilteringPass<T>::TextureFilteringPass(const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name)
    : texture_data_provider_(texture_data_provider)
    , feature_name_(feature_name)
{
}

template<typename T>
void TextureFilteringPass<T>::filter(std::vector<Candidate<T>>& candidates) const
{
    // There are only a few ranks, so a single pass that finds the highest one is enough
    std::vector<uint8_t> ranks;
    ranks.reserve(candidates.size());
    uint8_t highest_rank = 0;
    for (const Candidate<T>& candidate : candidates)
    {
        const uint8_t rank = computeRank(getElementPosition(*candidate.element));
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

template<typename T>
uint8_t TextureFilteringPass<T>::computeRank(const Point2LL& position) const
{
    const std::optional<TextureArea> preference = texture_data_provider_->getAreaPreference(position, feature_name_);
    if (preference.has_value())
    {
        switch (preference.value())
        {
        case TextureArea::Normal:
            return 1;
        case TextureArea::Preferred:
            return 2;
        case TextureArea::Avoid:
            return 0;
        }
    }

    return 1;
}

template class TextureFilteringPass<seam::SeamCandidate>;
template class TextureFilteringPass<Point2LL>;

} // namespace curaengine::types::scoring
