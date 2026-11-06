// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "bridge/BridgeAngleScoringCriterion.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;

/*! Scoring criterion for bridge angle candidate that bases the score on whether the angle is aligned with the shape of the surrounding area */
class AlignShapeScoringCriterion : public BridgeAngleScoringCriterion
{
public:
    explicit AlignShapeScoringCriterion(const std::vector<AngleDegrees>& candidates, const Shape& skin_outline, const Shape& supported_regions);

protected:
    double computeScore(const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area) const override;

private:
    std::optional<coord_t> total_segments_length_;
};

} // namespace cura
