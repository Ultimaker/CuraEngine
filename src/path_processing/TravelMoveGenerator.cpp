// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/TravelMoveGenerator.h"

namespace cura
{

TravelMoveGenerator::TravelMoveGenerator(const SpeedDerivatives& speed)
    : speed_(speed)
{
}

std::shared_ptr<TravelRoute> TravelMoveGenerator::generateTravelRoute(const Point3LL& start, const Point3LL& end) const
{
    return generateTravelRoute(start, end, speed_);
}

} // namespace cura
