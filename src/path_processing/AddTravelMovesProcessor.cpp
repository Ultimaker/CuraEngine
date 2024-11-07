// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/AddTravelMovesProcessor.h"

#include "path_planning/ExtruderMoveSequence.h"
#include "path_planning/ExtruderPlan.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_planning/TravelRoute.h"
#include "path_processing/DirectTravelMoveGenerator.h"

namespace cura
{

template<class OperationType, class ChildOperationType>
AddTravelMovesProcessor<OperationType, ChildOperationType>::AddTravelMovesProcessor(const SpeedDerivatives& speed)
    : speed_(speed)
{
    generators_.push_back(std::make_unique<DirectTravelMoveGenerator>(speed));
}

template<class OperationType, class ChildOperationType>
std::shared_ptr<PrintOperation> AddTravelMovesProcessor<OperationType, ChildOperationType>::makeOperation(
    const std::shared_ptr<ChildOperationType>& operation_before,
    const std::shared_ptr<ChildOperationType>& operation_after)
{
    if (const std::optional<Point3LL> start = findStartPosition(operation_before); start.has_value())
    {
        if (const std::optional<Point3LL> end = findEndPosition(operation_after); end.has_value())
        {
            if (end.value() != start.value())
            {
                for (const std::shared_ptr<TravelMoveGenerator>& generator : generators_)
                {
                    if (std::shared_ptr<TravelRoute> travel_route = generator->generateTravelRoute(start.value(), end.value()))
                    {
                        return travel_route;
                    }
                }
            }
        }
    }

    return nullptr;
}

template<>
std::optional<Point3LL> AddTravelMovesProcessor<ExtruderPlan, FeatureExtrusion>::findStartPosition(const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
{
    return feature_extrusion->findStartPosition();
}

template<>
std::optional<Point3LL> AddTravelMovesProcessor<ExtruderPlan, FeatureExtrusion>::findEndPosition(const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
{
    return feature_extrusion->findEndPosition();
}

template<>
std::optional<Point3LL> AddTravelMovesProcessor<FeatureExtrusion, ExtruderMoveSequence>::findStartPosition(const std::shared_ptr<ExtruderMoveSequence>& extruder_move_sequence)
{
    return extruder_move_sequence->getStartPosition();
}

template<>
std::optional<Point3LL> AddTravelMovesProcessor<FeatureExtrusion, ExtruderMoveSequence>::findEndPosition(const std::shared_ptr<ExtruderMoveSequence>& extruder_move_sequence)
{
    return extruder_move_sequence->findEndPosition();
}

template AddTravelMovesProcessor<ExtruderPlan, FeatureExtrusion>::AddTravelMovesProcessor(const SpeedDerivatives& speed);
template std::shared_ptr<PrintOperation> AddTravelMovesProcessor<ExtruderPlan, FeatureExtrusion>::makeOperation(
    const std::shared_ptr<FeatureExtrusion>& feature_extrusion_before,
    const std::shared_ptr<FeatureExtrusion>& feature_extrusion_after);

template AddTravelMovesProcessor<FeatureExtrusion, ExtruderMoveSequence>::AddTravelMovesProcessor(const SpeedDerivatives& speed);
template std::shared_ptr<PrintOperation> AddTravelMovesProcessor<FeatureExtrusion, ExtruderMoveSequence>::makeOperation(
    const std::shared_ptr<ExtruderMoveSequence>& feature_extrusion_before,
    const std::shared_ptr<ExtruderMoveSequence>& feature_extrusion_after);

} // namespace cura
