// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/TextureScoringCriterion.h"

#include <spdlog/spdlog.h>

#include "TextureDataProvider.h"


namespace cura
{

TextureScoringCriterion::TextureScoringCriterion(const PointsSet& points, const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name)
    : PositionBasedScoringCriterion(points)
    , texture_data_provider_(texture_data_provider)
    , feature_name_(feature_name)
{
}

double TextureScoringCriterion::computeScore(const Point2LL& candidate_position) const
{
    std::optional<TextureArea> preference = texture_data_provider_->getAreaPreference(candidate_position, feature_name_);
    if (preference.has_value())
    {
        switch (preference.value())
        {
        case TextureArea::Normal:
            return 0.5;
        case TextureArea::Preferred:
            return 1;
        case TextureArea::Avoid:
            return 0;
        }
    }

    return 0.5;
}

} // namespace cura
