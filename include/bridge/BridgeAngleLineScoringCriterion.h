#ifndef BRIDGE_BRIDGEANGLELINESCORINGCRITERION_H
#define BRIDGE_BRIDGEANGLELINESCORINGCRITERION_H

#include "bridge/BridgeAngleScoringCriterion.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;

class BridgeAngleLineScoringCriterion : public BridgeAngleScoringCriterion
{
public:
    explicit BridgeAngleLineScoringCriterion(const Shape& skin_outline, const Shape& supported_regions, coord_t line_width);
};

} // namespace cura

#endif // BRIDGE_BRIDGEANGLELINESCORINGCRITERION_H
