// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "bridge/BridgeAngleScoringCriterion.h"
#include "utils/Coord_t.h"

namespace curaengine::geometry
{
class Shape;
}

namespace curaengine::bridge
{


/*! Scoring criterion for bridge angle candidate that bases the score on whether the angle is aligned with the shape of the surrounding area */
class AlignShapeScoringCriterion : public BridgeAngleScoringCriterion
{
public:
    explicit AlignShapeScoringCriterion(const std::vector<types::AngleDegrees>& candidates, const geometry::Shape& skin_outline, const geometry::Shape& supported_regions);

protected:
    double computeScore(const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area) const override;

private:
    std::optional<utils::coord_t> total_segments_length_;
};

} // namespace curaengine::bridge
