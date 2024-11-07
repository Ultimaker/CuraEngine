// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_TRAVELMOVEGENERATOR_H
#define PATHPROCESSING_TRAVELMOVEGENERATOR_H

#include <memory>

#include "path_planning/SpeedDerivatives.h"

namespace cura
{

class Point3LL;
class TravelRoute;
class Settings;

class TravelMoveGenerator
{
public:
    explicit TravelMoveGenerator(const SpeedDerivatives& speed);

    virtual ~TravelMoveGenerator() = default; // Force class being polymorphic

    std::shared_ptr<TravelRoute> generateTravelRoute(const Point3LL& start, const Point3LL& end) const;

protected:
    virtual std::shared_ptr<TravelRoute> generateTravelRoute(const Point3LL& start, const Point3LL& end, const SpeedDerivatives& speed) const = 0;

private:
    const SpeedDerivatives& speed_;
};

} // namespace cura

#endif // PATHPROCESSING_TRAVELMOVEGENERATOR_H
