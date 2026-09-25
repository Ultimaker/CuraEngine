#ifndef BRIDGE_BRIDGEANGLESHAPESCORINGCRITERION_H
#define BRIDGE_BRIDGEANGLESHAPESCORINGCRITERION_H

#include "bridge/BridgeAngleScoringCriterion.h"

namespace cura
{

class Shape;

class BridgeAngleShapeScoringCriterion : public BridgeAngleScoringCriterion
{
public:
    explicit BridgeAngleShapeScoringCriterion(const Shape& bridging_area, const Shape& skin_outline);

    [[nodiscard]] static AngleDegrees preferredExtrusionAngle(const Shape& bridging_area, const Shape& skin_outline);
};

} // namespace cura

#endif // BRIDGE_BRIDGEANGLESHAPESCORINGCRITERION_H
