// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/TextureRankingCriterion.h"

#include <spdlog/spdlog.h>

#include "TextureArea.h"
#include "TextureDataProvider.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

TextureRankingCriterion::TextureRankingCriterion(const PointsSet& points, const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name)
    : PositionBasedRankingCriterion(points)
    , texture_data_provider_(texture_data_provider)
    , feature_name_(feature_name)
{
}

uint8_t TextureRankingCriterion::computeRank(const Point2LL& candidate_position) const
{
    std::optional<TextureArea> preference = texture_data_provider_->getAreaPreference(candidate_position, feature_name_);
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

} // namespace curaengine::types::scoring
