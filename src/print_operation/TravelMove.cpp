// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/TravelMove.h"

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
    const auto layer_plan = findParentByType<LayerPlan>();

    if (! travel_route || ! layer_plan)
    {
        return;
    }

    const Point3LL position = layer_plan->getAbsolutePosition(*travel_route, getPosition());
    const Velocity& velocity = travel_route->getSpeed();
    const PrintFeatureType feature = travel_route->getFeatureType();

    exporter.writeTravelMove(position, velocity, feature);
}

} // namespace cura