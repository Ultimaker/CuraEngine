// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/LayerChange.h"

#include "print_operation/TravelRoute.h"


namespace cura
{

LayerChange::LayerChange()
    : PrintOperationSequence()
{
}

void LayerChange::appendTravelRoute(const std::shared_ptr<TravelRoute>& travel_route)
{
    appendOperation(travel_route);
}

} // namespace cura
