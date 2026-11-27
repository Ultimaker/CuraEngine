// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/BridgeAngleScoringCriterion.h"

#include "types/geometry/PointMatrix.h"
#include "types/geometry/TransformedShape.h"


namespace curaengine::bridge
{
using namespace types::geometry;
using namespace types;

BridgeAngleScoringCriterion::BridgeAngleScoringCriterion(const Shape& skin_outline, const Shape& supported_regions)
    : skin_outline_(skin_outline)
    , supported_regions_(supported_regions)
{
}

double BridgeAngleScoringCriterion::computeScore(const AngleDegrees& angle) const
{
    const PointMatrix matrix(angle);
    const TransformedShape transformed_skin(skin_outline_, matrix);
    const TransformedShape transformed_supported_regions(supported_regions_, matrix);

    return computeScore(transformed_skin, transformed_supported_regions);
}

} // namespace curaengine::bridge
