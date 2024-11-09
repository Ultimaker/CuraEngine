// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/FeatureExtrusionsOrderOptimizer.h"

#include <spdlog/spdlog.h>

#include "path_planning/ContinuousExtruderMoveSequence.h"
#include "path_planning/ExtrusionMove.h"
#include "path_planning/FeatureExtrusion.h"

namespace cura
{

FeatureExtrusionsOrderOptimizer::FeatureExtrusionsOrderOptimizer(const Point3LL& previous_position)
    : previous_position_(previous_position)
{
}

void FeatureExtrusionsOrderOptimizer::process(ExtruderPlan* extruder_plan)
{
    std::vector<std::shared_ptr<FeatureExtrusion>> feature_extrusions = extruder_plan->getOperationsAs<FeatureExtrusion>();
    std::vector<std::shared_ptr<PrintOperation>> ordered_operations;
    ordered_operations.reserve(feature_extrusions.size());
    Point3LL current_position = previous_position_;

    { // First process the bed adhesion features
        auto iterator_end = std::partition(
            feature_extrusions.begin(),
            feature_extrusions.end(),
            [](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
            {
                return feature_extrusion->getPrintFeatureType() == PrintFeatureType::SkirtBrim;
            });

        for (auto iterator = feature_extrusions.begin(); iterator != iterator_end; ++iterator)
        {
            optimizeExtruderSequencesOrder(*iterator, current_position);
            ordered_operations.push_back(*iterator);
        }
        feature_extrusions.erase(feature_extrusions.begin(), iterator_end);
    }

    // For now, just add the others while just optimizing their sequences
    for (const std::shared_ptr<FeatureExtrusion>& operation : feature_extrusions)
    {
        optimizeExtruderSequencesOrder(operation, current_position);
        ordered_operations.push_back(operation);
    }
    extruder_plan->setOperations(ordered_operations);
}

void FeatureExtrusionsOrderOptimizer::optimizeExtruderSequencesOrder(const std::shared_ptr<FeatureExtrusion>& feature, Point3LL& current_position)
{
    std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>> moves_sequences = feature->getOperationsAs<ContinuousExtruderMoveSequence>();
    std::vector<std::shared_ptr<PrintOperation>> ordered_sequences;
    ordered_sequences.reserve(moves_sequences.size());

    enum class ChangeSequenceAction
    {
        None,
        Reverse,
        Reorder
    };

    struct ClosestPoint
    {
        coord_t distance_squared;
        std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>>::iterator move_sequence_iterator;
        std::shared_ptr<ExtrusionMove> move; // The move containing the target position, or null for the actual starting point
        ChangeSequenceAction action;
    };

    auto evaluate_closest_point = [&current_position](
                                      const Point3LL& point,
                                      std::optional<ClosestPoint>& closest_point,
                                      const std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>>::iterator& move_sequence,
                                      const std::shared_ptr<ExtrusionMove>& move,
                                      const ChangeSequenceAction action)
    {
        const coord_t distance_squared = (point - current_position).vSize2();
        if (! closest_point.has_value() || distance_squared < closest_point->distance_squared)
        {
            closest_point = ClosestPoint{ distance_squared, move_sequence, move, action };
        }
    };

#warning This is extremely unoptimized, just good enough for the POC
    while (! moves_sequences.empty())
    {
        std::optional<ClosestPoint> closest_point;

        for (auto iterator = moves_sequences.begin(); moves_sequences.end() != iterator; ++iterator)
        {
            const std::shared_ptr<ContinuousExtruderMoveSequence>& move_sequence = *iterator;
            if (move_sequence->isClosed())
            {
                for (const std::shared_ptr<ExtrusionMove>& extrusion_move : move_sequence->getOperationsAs<ExtrusionMove>())
                {
                    evaluate_closest_point(extrusion_move->getPosition(), closest_point, iterator, extrusion_move, ChangeSequenceAction::Reorder);
                }
            }
            else
            {
                if (std::optional<Point3LL> start_position = move_sequence->findStartPosition(); start_position.has_value())
                {
                    evaluate_closest_point(start_position.value(), closest_point, iterator, nullptr, ChangeSequenceAction::None);
                }

                if (std::optional<Point3LL> end_position = move_sequence->findEndPosition(); end_position.has_value())
                {
                    evaluate_closest_point(end_position.value(), closest_point, iterator, nullptr, ChangeSequenceAction::Reverse);
                }
            }
        }

        if (closest_point.has_value())
        {
            switch (closest_point->action)
            {
            case ChangeSequenceAction::None:
                break;
            case ChangeSequenceAction::Reorder:
                if (closest_point->move)
                {
                    (*closest_point->move_sequence_iterator)->reorderToEndWith(closest_point->move);
                }
                break;
            case ChangeSequenceAction::Reverse:
                (*closest_point->move_sequence_iterator)->reverse();
            }

            current_position = (*closest_point->move_sequence_iterator)->findEndPosition().value();
            ordered_sequences.push_back(*closest_point->move_sequence_iterator);
            moves_sequences.erase(closest_point->move_sequence_iterator);
        }
        else
        {
            spdlog::error("If this happens, we are all gonna die quite soon, call your family and tell them you love them. Well, do it anyway, we are still gonna die someday, so "
                          "take care and enjoy.");
            break; // Try to save the world anyway, it's worth it
        }
    }

    feature->setOperations(ordered_sequences);
}

} // namespace cura
