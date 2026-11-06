// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "settings/types/Angle.h"
#include "utils/scoring/ScoringCriterion.h"

namespace cura
{

struct AngleCandidate;
class TransformedShape;
class Shape;

/*! Base class for criteria that calculate a score for a candidate bridging angle */
class BridgeAngleScoringCriterion : public ScoringCriterion
{
public:
    explicit BridgeAngleScoringCriterion(const std::vector<AngleDegrees>& candidates, const Shape& skin_outline, const Shape& supported_regions);

    double computeScore(const size_t candidate_index) const override;

protected:
    /*!
     * Method to be overridden by child classes to actually calculate the score of an angle candidate
     * @param transformed_skin The skin area, rotated so that it is in the plan where bridging lines are horizontal
     * @param transformed_supported_regions The supported regions, rotated so that it is in the plan where bridging lines are horizontal
     * @return The score of this candidate
     */
    virtual double computeScore(const TransformedShape& transformed_skin, const TransformedShape& transformed_supported_regions) const = 0;

private:
    const std::vector<AngleDegrees>& candidates_;
    const Shape& skin_outline_;
    const Shape& supported_regions_;
};

} // namespace cura
