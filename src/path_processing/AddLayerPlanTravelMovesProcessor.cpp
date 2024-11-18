// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/AddLayerPlanTravelMovesProcessor.h"

#include "path_planning/ContinuousExtruderMoveSequence.h"
#include "path_planning/ExtruderPlan.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_planning/TravelRoute.h"
#include "path_processing/DirectTravelMoveGenerator.h"

namespace cura
{

AddLayerPlanTravelMovesProcessor::AddLayerPlanTravelMovesProcessor()
{
    generators_.push_back(std::make_shared<DirectTravelMoveGenerator>());
}

void AddLayerPlanTravelMovesProcessor::process(LayerPlan* layer_plan)
{
    // First, append "regular" travels moves between features and extrusion sequences of the same extruder plan
    for (const std::shared_ptr<ExtruderPlan>& extruder_plan : layer_plan->getOperationsAs<ExtruderPlan>())
    {
        const SpeedDerivatives& speed = extruder_plan->getTravelSpeed();
        appendTravelMovesRecursively(extruder_plan, speed);
    }

    // Now we should link extruder plans to each other, but that has to take care of the tool change sequence
}

void AddLayerPlanTravelMovesProcessor::appendTravelsMovesBetweenChildren(const std::shared_ptr<PrintOperationSequence>& sequence, const SpeedDerivatives& speed)
{
    std::vector<std::shared_ptr<PrintOperation>>& child_operations = sequence->getOperations();
    for (size_t index_first = 0; index_first < child_operations.size(); ++index_first)
    {
        const std::shared_ptr<PrintOperation>& operation_first = child_operations[index_first];
        std::optional<Point3LL> first_end_position = operation_first->findEndPosition();
        if (first_end_position.has_value())
        {
            for (size_t index_second = index_first + 1; index_second < child_operations.size(); ++index_second)
            {
                const std::shared_ptr<PrintOperation>& operation_second = child_operations[index_second];
                std::optional<Point3LL> second_start_position = operation_second->findStartPosition();
                if (second_start_position.has_value())
                {
                    if (const std::shared_ptr<PrintOperation> travel_move = makeTravelMove(first_end_position.value(), second_start_position.value(), speed))
                    {
                        child_operations.insert(std::next(child_operations.begin(), index_second), travel_move);
                        index_first = index_second;
                    }
                    else
                    {
                        index_first = index_second - 1;
                    }
                    break;
                }
            }
        }
    }
}

void AddLayerPlanTravelMovesProcessor::appendTravelMovesRecursively(const std::shared_ptr<PrintOperation>& operation, const SpeedDerivatives& speed)
{
    if (const auto operation_sequence = std::dynamic_pointer_cast<PrintOperationSequence>(operation))
    {
        for (const auto& child_operation : operation_sequence->getOperations())
        {
            appendTravelMovesRecursively(child_operation, speed);
        }

        appendTravelsMovesBetweenChildren(operation_sequence, speed);
    }
}

const std::shared_ptr<PrintOperation> AddLayerPlanTravelMovesProcessor::makeTravelMove(const Point3LL& start_position, const Point3LL& end_position, const SpeedDerivatives& speed)
{
    if (end_position != start_position)
    {
        for (const std::shared_ptr<TravelMoveGenerator>& generator : generators_)
        {
            if (std::shared_ptr<TravelRoute> travel_route = generator->generateTravelRoute(start_position, end_position, speed))
            {
                return travel_route;
            }
        }
    }

    return nullptr;
}

} // namespace cura
