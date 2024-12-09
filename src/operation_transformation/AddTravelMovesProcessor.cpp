// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/AddTravelMovesProcessor.h"

#include "operation_transformation/DirectTravelMoveGenerator.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/TravelRoute.h"

namespace cura
{

AddTravelMovesProcessor::AddTravelMovesProcessor(const SpeedDerivatives& speed)
    : speed_(speed)
{
    // generators_.push_back(std::make_unique<DirectTravelMoveGenerator>(speed));
}

bool AddTravelMovesProcessor::firstOperationMatches(const std::shared_ptr<PrintOperation>& operation)
{
    return operation->findEndPosition().has_value();
}

bool AddTravelMovesProcessor::secondOperationMatches(const std::shared_ptr<PrintOperation>& /*first_operation*/, const std::shared_ptr<PrintOperation>& operation)
{
    return operation->findStartPosition().has_value();
}

std::shared_ptr<PrintOperation>
    AddTravelMovesProcessor::makeOperation(const std::shared_ptr<PrintOperation>& operation_first, const std::shared_ptr<PrintOperation>& operation_second)
{
    const Point3LL start = operation_first->findEndPosition().value();
    const Point3LL end = operation_second->findStartPosition().value();
    if (end != start)
    {
        for (const std::shared_ptr<TravelMoveGenerator>& generator : generators_)
        {
            // if (std::shared_ptr<TravelRoute> travel_route = generator->generateTravelRoute(start, end))
            // {
            //     return travel_route;
            // }
        }
    }

    return nullptr;
}

} // namespace cura