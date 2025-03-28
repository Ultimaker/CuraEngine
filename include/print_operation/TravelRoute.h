// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_TRAVELROUTE_H
#define PATHPLANNING_TRAVELROUTE_H

#include "path_planning/SpeedDerivatives.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"

namespace cura
{

enum class PrintFeatureType : uint8_t;
class TravelMove;
struct Velocity;

class TravelRoute : public ContinuousExtruderMoveSequence
{
public:
    explicit TravelRoute(const PrintFeatureType feature, const SpeedDerivatives& speed, const Point3LL& start_position);

    void appendTravelMove(const std::shared_ptr<TravelMove>& travel_move);

    const Velocity& getSpeed() const;

    PrintFeatureType getFeatureType() const;

private:
    const PrintFeatureType feature_;
    const SpeedDerivatives speed_;
};

} // namespace cura

#endif // PATHPLANNING_TRAVELROUTE_H
