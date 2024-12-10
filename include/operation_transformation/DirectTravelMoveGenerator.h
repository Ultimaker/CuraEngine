// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "operation_transformation/TravelMoveGenerator.h"

namespace cura
{

class DirectTravelMoveGenerator : public TravelMoveGenerator
{
public:
    std::shared_ptr<TravelRoute> generateTravelRoute(const Point3LL& start, const Point3LL& end, const SpeedDerivatives& speed) const override;
};

} // namespace cura
