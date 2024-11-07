// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_DIRECTTRAVELMOVEGENERATOR_H
#define PATHPROCESSING_DIRECTTRAVELMOVEGENERATOR_H

#include "path_processing/TravelMoveGenerator.h"

namespace cura
{

class DirectTravelMoveGenerator : public TravelMoveGenerator
{
public:
    DirectTravelMoveGenerator(const SpeedDerivatives& speed);

protected:
    std::shared_ptr<TravelRoute> generateTravelRoute(const Point3LL& start, const Point3LL& end, const SpeedDerivatives& speed) const override;
};

} // namespace cura

#endif // PATHPROCESSING_DIRECTTRAVELMOVEGENERATOR_H
