// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/ExtruderPlanScheduler.h"

#include <range/v3/view/filter.hpp>
#include <spdlog/spdlog.h>

#include "path_planning/ContinuousExtruderMoveSequence.h"
#include "path_planning/ExtrusionMove.h"
#include "path_processing/ClosestStartPoint.h"
#include "path_processing/FeatureExtrusionScheduler.h"
#include "settings/ZSeamConfig.h"

namespace cura
{

ExtruderPlanScheduler::ExtruderPlanScheduler()
{
}

void ExtruderPlanScheduler::process(ExtruderPlan* extruder_plan)
{
    std::map<FeatureExtrusionPtr, std::shared_ptr<FeatureExtrusionScheduler>> feature_extrusion_schedulers;

    { // Make schedulers for individual feature extrusions
        std::vector<std::shared_ptr<FeatureExtrusion>> feature_extrusions = extruder_plan->getOperationsAs<FeatureExtrusion>();
        for (const std::shared_ptr<FeatureExtrusion>& feature_extrusion : feature_extrusions)
        {
            feature_extrusion_schedulers[feature_extrusion] = std::make_shared<FeatureExtrusionScheduler>(feature_extrusion, feature_extrusions);
        }
    }

    // Make a view containing the features for easier iteration
    auto feature_extrusions = feature_extrusion_schedulers | std::views::keys;
    auto feature_schedulers = feature_extrusion_schedulers | std::views::values;

    std::vector<std::shared_ptr<PrintOperation>> ordered_operations;
    ordered_operations.reserve(feature_extrusion_schedulers.size());

    const auto is_feature_processable_now = [&feature_schedulers](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
    {
        return std::ranges::all_of(
            feature_schedulers,
            [&feature_extrusion](const std::shared_ptr<FeatureExtrusionScheduler>& feature_extrusion_scheduler)
            {
                return feature_extrusion_scheduler->isFeatureProcessableNow(feature_extrusion);
            });
    };

    // Now loop until we have ordered all the features
    while (! feature_extrusion_schedulers.empty())
    {
        // TODO: this can probably be optimized
        // Evaluate the candidate points for every feature that can be processed now
        std::optional<ClosestStartPoint> closest_point;
        for (const auto& [feature, scheduler] : feature_extrusion_schedulers)
        {
            if (is_feature_processable_now(feature))
            {
                scheduler->evaluateClosestPoint(closest_point, current_position_);
            }
        }

        if (closest_point.has_value())
        {
            // Optimizer inner sequences or feature, starting by the sequence holding the best candidate, and update current position for next candidate selection
            const StartCandidatePoint& best_candidate = closest_point->point;
            auto iterator = feature_extrusion_schedulers.find(best_candidate.feature_extrusion);
            assert(iterator != feature_extrusion_schedulers.end() && "Feature not found in map");

            // Now apply the change to the move sequence to have it start with the given point
            iterator->second->optimizeExtruderSequencesOrder(best_candidate, current_position_);

            // Register this feature as being processed next
            ordered_operations.push_back(best_candidate.feature_extrusion);

            // Remove everything related to this feature
            feature_extrusion_schedulers.erase(iterator);
        }
        else
        {
            spdlog::error("If this happens, we are all gonna die quite soon, call your family and tell them you love them. Well, do it anyway, we are still gonna die someday, so "
                          "take care and enjoy.");
            break; // Try to save the world anyway, it's worth it
        }
    }

    extruder_plan->setOperations(ordered_operations);
}

// void ExtruderPlanScheduler::optimizeExtruderSequencesOrder(
//     const std::shared_ptr<FeatureExtrusion>& feature,
//     const std::shared_ptr<ContinuousExtruderMoveSequence>& start_sequence,
//     std::vector<StartCandidatePoint> start_candidates)
// {
//     std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>> moves_sequences = feature->getOperationsAs<ContinuousExtruderMoveSequence>();
//     std::vector<std::shared_ptr<PrintOperation>> ordered_sequences;
//     ordered_sequences.reserve(moves_sequences.size());
//     ordered_sequences.push_back(start_sequence);
//     std::erase(moves_sequences, start_sequence);
//     std::erase_if(
//         start_candidates,
//         [&start_sequence](const StartCandidatePoint& candidate_point)
//         {
//             return candidate_point.move_sequence == start_sequence;
//         });
//
//     std::optional<Point3LL> start_sequence_end_position = start_sequence->findEndPosition();
//     assert(start_sequence_end_position.has_value() && "Unable to find the end position of the given start sequence");
//     current_position_ = start_sequence_end_position.value();
//
//     // TODO: this can probably be optimized
//     while (! moves_sequences.empty())
//     {
//         std::optional<ClosestPoint> closest_point;
//         findClosestPoint(start_candidates, closest_point);
//
//         if (closest_point.has_value())
//         {
//             const StartCandidatePoint& best_candidate = closest_point->point;
//             applyMoveSequenceAction(best_candidate.move_sequence, best_candidate.move, best_candidate.action);
//
//             std::optional<Point3LL> end_position = best_candidate.move_sequence->findEndPosition().value();
//             assert(end_position.has_value() && "The move sequence doesn't have an end position");
//             current_position_ = end_position.value();
//             ordered_sequences.push_back(best_candidate.move_sequence);
//             std::erase(moves_sequences, best_candidate.move_sequence);
//
//             std::erase_if(
//                 start_candidates,
//                 [&best_candidate](const StartCandidatePoint& candidate_point)
//                 {
//                     return candidate_point.move_sequence == best_candidate.move_sequence;
//                 });
//         }
//         else
//         {
//             spdlog::error("If this happens, we are all gonna die quite soon, call your family and tell them you love them. Well, do it anyway, we are still gonna die someday, so
//             "
//                           "take care and enjoy.");
//             break; // Try to save the world anyway, it's worth it
//         }
//     }
//
//     feature->setOperations(ordered_sequences);
// }

} // namespace cura
