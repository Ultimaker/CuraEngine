// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/TravelRoute.h"

#include "print_operation/TravelMove.h"

namespace cura
{

TravelRoute::TravelRoute(const PrintFeatureType feature, const SpeedDerivatives& speed, const Point3LL& start_position)
    : ContinuousExtruderMoveSequence(false, start_position)
    , feature_(feature)
    , speed_(speed)
{
}

void TravelRoute::appendTravelMove(const std::shared_ptr<TravelMove>& travel_move)
{
    appendOperation(travel_move);
}

const Velocity& TravelRoute::getSpeed() const
{
    return speed_.speed;
}

PrintFeatureType TravelRoute::getFeatureType() const
{
    return feature_;
}

} // namespace cura
