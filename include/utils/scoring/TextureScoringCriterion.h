// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEXTURESCORINGCRITERION_H
#define TEXTURESCORINGCRITERION_H

#include <memory>

#include "utils/scoring/PositionBasedScoringCriterion.h"

namespace cura
{
class TextureDataProvider;

/*!
 * Criterion that will give a score according to the data contained in the user-painted texture.
 */
class TextureScoringCriterion : public PositionBasedScoringCriterion
{
private:
    const std::shared_ptr<TextureDataProvider> texture_data_provider_;
    const std::string feature_name_;

public:
    explicit TextureScoringCriterion(const PointsSet& points, const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name);

    virtual double computeScore(const Point2LL& candidate_position) const override;
};

} // namespace cura

#endif // TEXTURESCORINGCRITERION_H
