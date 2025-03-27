// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class TravelRoute;

class LayerChange : public PrintOperationSequence
{
public:
    explicit LayerChange();

    void appendTravelRoute(const std::shared_ptr<TravelRoute>& travel_route);
};

} // namespace cura
