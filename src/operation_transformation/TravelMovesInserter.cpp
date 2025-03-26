// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/TravelMovesInserter.h"

#include <range/v3/view/sliding.hpp>

#include "Application.h"
#include "Slice.h"
#include "operation_transformation/DirectTravelMoveGenerator.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/TravelRoute.h"

namespace cura
{

TravelMovesInserter::TravelMovesInserter()
{
    generators_.push_back(std::make_shared<DirectTravelMoveGenerator>());
}

void TravelMovesInserter::process(PrintPlan* print_plan)
{
    const size_t initial_speedup_layer_count = Application::getInstance().current_slice_->scene.settings.get<size_t>("speed_slowdown_layers");
    const std::vector<ExtruderNumber> used_extruders = print_plan->calculateUsedExtruders();
    const std::map<ExtruderNumber, SpeedDerivatives> extruders_travel_speed_initial = makeTravelSpeedsInitial(used_extruders);
    const std::map<ExtruderNumber, SpeedDerivatives> extruders_travel_speed_up = makeTravelSpeedsUp(used_extruders);

    for (const LayerPlanPtr& layer_plan : print_plan->getOperationsAs<LayerPlan>())
    {
        std::vector<ExtruderPlanPtr> extruder_plans = layer_plan->getOperationsAs<ExtruderPlan>();

        // First, append "regular" travel moves between features and extrusion sequences of the same extruder plan
        for (const ExtruderPlanPtr& extruder_plan : extruder_plans)
        {
            const SpeedDerivatives speed = getTravelSpeed(
                extruders_travel_speed_initial,
                extruders_travel_speed_up,
                initial_speedup_layer_count,
                layer_plan->getLayerIndex(),
                extruder_plan->getExtruderNr());
            appendTravelMovesRecursively(extruder_plan, speed);
        }

        // Now link extruder plans to each other
        for (const auto& extruder_plans_window : extruder_plans | ranges::views::sliding(2))
        {
            const ExtruderPlanPtr extruder_plan_before = extruder_plans_window[0];
            const std::optional<Point3LL> end_position_before = extruder_plan_before->findEndPosition();

            const ExtruderPlanPtr extruder_plan_after = extruder_plans_window[1];
            const std::optional<Point3LL> start_position_after = extruder_plan_after->findStartPosition();

            if (end_position_before.has_value() && start_position_after.has_value())
            {
                const SpeedDerivatives speed = getTravelSpeed(
                    extruders_travel_speed_initial,
                    extruders_travel_speed_up,
                    initial_speedup_layer_count,
                    layer_plan->getLayerIndex(),
                    extruder_plan_before->getExtruderNr());

                if (const std::shared_ptr<TravelRoute> travel_move = makeTravelRoute(end_position_before.value(), start_position_after.value(), speed))
                {
                    layer_plan->insertTravelRouteAfter(travel_move, extruder_plan_before);
                }
            }
        }
    }
}

void TravelMovesInserter::appendTravelsMovesBetweenChildren(const std::shared_ptr<PrintOperationSequence>& sequence, const SpeedDerivatives& speed)
{
    std::vector<std::shared_ptr<PrintOperation>> child_operations = sequence->getOperations();
    for (size_t index_first = 0; index_first < child_operations.size() - 1; ++index_first)
    {
        const std::shared_ptr<PrintOperation>& operation_first = child_operations[index_first];
        std::optional<Point3LL> first_end_position = operation_first->findEndPosition();
        if (! first_end_position.has_value())
        {
            continue;
        }

        for (size_t index_second = index_first + 1; index_second < child_operations.size(); ++index_second)
        {
            const std::shared_ptr<PrintOperation>& operation_second = child_operations[index_second];
            std::optional<Point3LL> second_start_position = operation_second->findStartPosition();
            if (! second_start_position.has_value())
            {
                continue;
            }

            if (const std::shared_ptr<TravelRoute> travel_move = makeTravelRoute(first_end_position.value(), second_start_position.value(), speed))
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

    sequence->setOperations(child_operations);
}

void TravelMovesInserter::appendTravelMovesRecursively(const std::shared_ptr<PrintOperation>& operation, const SpeedDerivatives& speed)
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

const std::shared_ptr<TravelRoute> TravelMovesInserter::makeTravelRoute(const Point3LL& start_position, const Point3LL& end_position, const SpeedDerivatives& speed)
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

const SpeedDerivatives TravelMovesInserter::getTravelSpeed(
    const std::map<ExtruderNumber, SpeedDerivatives>& extruders_travel_speed_initial,
    const std::map<ExtruderNumber, SpeedDerivatives>& extruders_travel_speed_up,
    const size_t initial_speedup_layer_count,
    const LayerIndex& layer_nr,
    const ExtruderNumber extruder_nr) const
{
    if (layer_nr <= 0)
    {
        return extruders_travel_speed_initial.at(extruder_nr);
    }

    if (layer_nr >= initial_speedup_layer_count)
    {
        return extruders_travel_speed_up.at(extruder_nr);
    }

    SpeedDerivatives speed = extruders_travel_speed_up.at(extruder_nr);
    speed.smoothSpeed(extruders_travel_speed_initial.at(extruder_nr), std::max(LayerIndex(0), layer_nr), initial_speedup_layer_count);
    return speed;
}

std::map<ExtruderNumber, SpeedDerivatives> TravelMovesInserter::makeTravelSpeedsInitial(const std::vector<ExtruderNumber>& used_extruders) const
{
    return makeTravelSpeeds(used_extruders, "speed_travel_layer_0", "acceleration_travel_layer_0", "jerk_travel_layer_0");
}

std::map<ExtruderNumber, SpeedDerivatives> TravelMovesInserter::makeTravelSpeedsUp(const std::vector<ExtruderNumber>& used_extruders) const
{
    return makeTravelSpeeds(used_extruders, "speed_travel", "acceleration_travel", "jerk_travel");
}

std::map<ExtruderNumber, SpeedDerivatives> TravelMovesInserter::makeTravelSpeeds(
    const std::vector<ExtruderNumber>& used_extruders,
    const std::string& setting_speed,
    const std::string& setting_acceleration,
    const std::string& setting_jerk) const
{
    std::map<ExtruderNumber, SpeedDerivatives> extruders_travel_speed;

    for (const ExtruderNumber extruder_nr : used_extruders)
    {
        const Settings& settings = Application::getInstance().current_slice_->scene.getExtruder(extruder_nr).settings_;
        extruders_travel_speed[extruder_nr] = SpeedDerivatives{ .speed = settings.get<Velocity>(setting_speed),
                                                                .acceleration = settings.get<Acceleration>(setting_acceleration),
                                                                .jerk = settings.get<Velocity>(setting_jerk) };
    }

    return extruders_travel_speed;
}

} // namespace cura
