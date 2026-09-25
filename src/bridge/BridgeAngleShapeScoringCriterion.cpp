#include "bridge/BridgeAngleShapeScoringCriterion.h"

#include <algorithm>

#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{
namespace
{

std::vector<double> buildShapeScores(const Shape& bridging_area, const Shape& skin_outline)
{
    Shape preferred_shape = bridging_area.empty() ? skin_outline : bridging_area;
    if (preferred_shape.empty())
    {
        return std::vector<double>(BridgeAngleScoringCriterion::candidatesCount(), 1.0);
    }

    preferred_shape.makeConvex();
    const auto [aabb, angle] = AABB::minimumAreaOrientedBoundingBox(preferred_shape);

    AngleDegrees preferred_axis_angle(angle);
    if (aabb.height() > aabb.width())
    {
        preferred_axis_angle += 90;
    }
    const AngleDegrees preferred_extrusion_angle(-static_cast<double>(preferred_axis_angle));

    const coord_t longest_side = std::max(aabb.width(), aabb.height());
    const coord_t shortest_side = std::min(aabb.width(), aabb.height());
    const double preference_strength = longest_side > 0 ? 1.0 - static_cast<double>(shortest_side) / longest_side : 0.0;

    std::vector<double> scores;
    scores.reserve(BridgeAngleScoringCriterion::candidatesCount());
    for (size_t candidate_index = 0; candidate_index < BridgeAngleScoringCriterion::candidatesCount(); ++candidate_index)
    {
        const double delta = BridgeAngleScoringCriterion::extrusionAngleDistance(
            BridgeAngleScoringCriterion::candidateIndexToExtrusionAngle(candidate_index),
            preferred_extrusion_angle);
        scores.push_back(1.0 - preference_strength * delta / 90.0);
    }

    return scores;
}

} // namespace

BridgeAngleShapeScoringCriterion::BridgeAngleShapeScoringCriterion(const Shape& bridging_area, const Shape& skin_outline)
    : BridgeAngleScoringCriterion(buildShapeScores(bridging_area, skin_outline))
{
}

} // namespace cura
