// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/DirectTravelMoveGenerator.h"

#include "path_planning/TravelMove.h"
#include "path_planning/TravelRoute.h"

namespace cura
{

std::shared_ptr<TravelRoute> DirectTravelMoveGenerator::generateTravelRoute(const Point3LL& start, const Point3LL& end, const SpeedDerivatives& speed) const
{
    auto route = std::make_shared<TravelRoute>(PrintFeatureType::MoveRetraction, speed, start);
    route->appendTravelMove(std::make_shared<TravelMove>(end));
    return route;
}
} // namespace cura
