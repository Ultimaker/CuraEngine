// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>

#include "types/scoring/PositionBasedRankingCriterion.h"

namespace curaengine
{
class TextureDataProvider;
}

namespace curaengine::types::scoring
{


/*!
 * Criterion that will give a rank according to the data contained in the user-painted texture.
 */
class TextureRankingCriterion : public PositionBasedRankingCriterion
{
private:
    const std::shared_ptr<TextureDataProvider> texture_data_provider_;
    const std::string feature_name_;

public:
    explicit TextureRankingCriterion(const types::geometry::PointsSet& points, const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name);

    virtual uint8_t computeRank(const types::geometry::Point2LL& candidate_position) const override;
};

} // namespace curaengine::types::scoring
