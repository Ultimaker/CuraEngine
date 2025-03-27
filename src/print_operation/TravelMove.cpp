// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/TravelMove.h"

#include "plan_export/PlanExporter.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/TravelRoute.h"

namespace cura
{

TravelMove::TravelMove(const Point3LL& position)
    : ExtruderMove(position)
{
}

void TravelMove::write(PlanExporter& exporter) const
{
    const auto travel_route = findParentByType<TravelRoute>();
    if (! travel_route)
    {
        spdlog::error("TravelMove is not part of a TravelRoute");
        return;
    }

    const Point3LL position = getAbsolutePosition();
    const Velocity& velocity = travel_route->getSpeed();
    const PrintFeatureType feature = travel_route->getFeatureType();

    exporter.writeTravelMove(position, velocity, feature);
}

} // namespace cura
