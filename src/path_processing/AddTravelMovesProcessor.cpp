// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/AddTravelMovesProcessor.h"

#include "path_planning/ExtruderPlan.h"
#include "path_planning/TravelRoute.h"
#include "path_processing/DirectTravelMoveGenerator.h"

namespace cura
{

AddTravelMovesProcessor::AddTravelMovesProcessor(const SpeedDerivatives& speed)
    : speed_(speed)
{
    generators_.push_back(std::make_unique<DirectTravelMoveGenerator>(speed));
}

std::shared_ptr<PrintOperation>
    AddTravelMovesProcessor::makeOperation(const std::shared_ptr<FeatureExtrusion>& feature_extrusion_before, const std::shared_ptr<FeatureExtrusion>& feature_extrusion_after)
{
    if (const std::optional<Point3LL> start = feature_extrusion_before->findEndPosition(); start.has_value())
    {
        const Point3LL& end = feature_extrusion_after->getStartPosition();

        for (const std::shared_ptr<TravelMoveGenerator>& generator : generators_)
        {
            if (std::shared_ptr<TravelRoute> travel_route = generator->generateTravelRoute(start.value(), end))
            {
                return travel_route;
            }
        }
    }

    return nullptr;
}

} // namespace cura
