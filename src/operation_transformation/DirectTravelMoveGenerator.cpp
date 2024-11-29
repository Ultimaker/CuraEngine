// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/DirectTravelMoveGenerator.h"

#include "print_operation/TravelMove.h"
#include "print_operation/TravelRoute.h"

namespace cura
{

std::shared_ptr<TravelRoute> DirectTravelMoveGenerator::generateTravelRoute(const Point3LL& start, const Point3LL& end, const SpeedDerivatives& speed) const
{
    auto route = std::make_shared<TravelRoute>(PrintFeatureType::MoveRetraction, speed, start);
    route->appendTravelMove(std::make_shared<TravelMove>(end));
    return route;
}
} // namespace cura
