// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/BridgeAngleScoringCriterion.h"

#include "bridge/TransformedShape.h"
#include "geometry/PointMatrix.h"


namespace curaengine::bridge
{

BridgeAngleScoringCriterion::BridgeAngleScoringCriterion(const std::vector<::curaengine::types::AngleDegrees>& candidates, const ::curaengine::geometry::Shape& skin_outline, const ::curaengine::geometry::Shape& supported_regions)
    : candidates_(candidates)
    , skin_outline_(skin_outline)
    , supported_regions_(supported_regions)
{
}

double BridgeAngleScoringCriterion::computeScore(const size_t candidate_index) const
{
    const ::curaengine::types::AngleDegrees& angle = candidates_[candidate_index];
    const ::curaengine::geometry::PointMatrix matrix(angle);
    const TransformedShape transformed_skin(skin_outline_, matrix);
    const TransformedShape transformed_supported_regions(supported_regions_, matrix);

    return computeScore(transformed_skin, transformed_supported_regions);
}

} // namespace curaengine::bridge
