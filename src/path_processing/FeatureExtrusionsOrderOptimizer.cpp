// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/FeatureExtrusionsOrderOptimizer.h"

#include <algorithm>

#include <range/v3/view/filter.hpp>
#include <spdlog/spdlog.h>

#include "path_planning/ContinuousExtruderMoveSequence.h"
#include "path_planning/ExtrusionMove.h"
#include "path_planning/FeatureExtrusion.h"

namespace cura
{

void FeatureExtrusionsOrderOptimizer::process(ExtruderPlan* extruder_plan)
{
    std::vector<std::shared_ptr<FeatureExtrusion>> feature_extrusions = extruder_plan->getOperationsAs<FeatureExtrusion>();
    std::vector<std::shared_ptr<PrintOperation>> ordered_operations;
    ordered_operations.reserve(feature_extrusions.size());

    // First, register all the possible starting positions amongst all the extrusion features
    std::map<std::shared_ptr<FeatureExtrusion>, std::vector<StartCandidatePoint>> start_candidates = makeStartCandidates(feature_extrusions);

    // Now create the ordering constraints
    std::vector<FeatureExtrusionOrderingConstraint> feature_extrusions_constraints = makeFeatureExtrusionOrderingConstraints(feature_extrusions);
    // std::vector<MoveSequenceOrderingConstraint> move_sequences_constraints = makeMoveSequenceOrderingConstraints(feature_extrusions);

    const auto is_feature_doable_now = [&feature_extrusions_constraints](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
    {
        return std::ranges::find_if(
                   feature_extrusions_constraints,
                   [&feature_extrusion](const FeatureExtrusionOrderingConstraint& constraint)
                   {
                       return constraint.feature_after == feature_extrusion;
                   })
            == feature_extrusions_constraints.end();
    };

    // Now loop until we have ordered all the features
    while (! feature_extrusions.empty())
    {
#warning This can probably be optimized, but good enough for the POC
        // Evaluate the candidate points for every feature that can be processed now
        std::optional<ClosestPoint> closest_point;
        for (const std::shared_ptr<FeatureExtrusion>& feature_extrusion : feature_extrusions | ranges::views::filter(is_feature_doable_now))
        {
            for (const StartCandidatePoint& start_candidate : start_candidates[feature_extrusion])
            {
                const coord_t distance_squared = (start_candidate.position - current_position_).vSize2();
                if (! closest_point.has_value() || distance_squared < closest_point->distance_squared)
                {
                    closest_point = ClosestPoint{ distance_squared, start_candidate };
                }
            }
        }

        if (closest_point.has_value())
        {
            // Now apply the change to the move sequence to have it start with the given point
            const StartCandidatePoint& best_candidate = closest_point->point;
            applyMoveSequenceAction(best_candidate.move_sequence, best_candidate.move, best_candidate.action);

            // Optimizer inner sequences or feature, starting by the sequence holding the best candidate, and update current position for next candidate selection
            current_position_ = optimizeExtruderSequencesOrder(best_candidate.feature_extrusion, best_candidate.move_sequence, start_candidates[best_candidate.feature_extrusion]);

            // Register this feature as being processed next
            ordered_operations.push_back(best_candidate.feature_extrusion);

            // Remove processed feature from processing list
            std::erase(feature_extrusions, best_candidate.feature_extrusion);

            // Remove constraints related to this feature
            std::erase_if(
                feature_extrusions_constraints,
                [&best_candidate](const FeatureExtrusionOrderingConstraint& constraint)
                {
                    return constraint.feature_before == best_candidate.feature_extrusion;
                });

            // Remove start candidates from this feature
            auto iterator = start_candidates.find(best_candidate.feature_extrusion);
            if (iterator != start_candidates.end())
            {
                start_candidates.erase(iterator);
            }
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

std::map<std::shared_ptr<FeatureExtrusion>, std::vector<FeatureExtrusionsOrderOptimizer::StartCandidatePoint>>
    FeatureExtrusionsOrderOptimizer::makeStartCandidates(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const
{
    std::map<std::shared_ptr<FeatureExtrusion>, std::vector<StartCandidatePoint>> start_candidates;

    for (const std::shared_ptr<FeatureExtrusion>& feature_extrusion : feature_extrusions)
    {
        std::vector<StartCandidatePoint> feature_start_candidates;

        for (const std::shared_ptr<ContinuousExtruderMoveSequence>& move_sequence : feature_extrusion->getOperationsAs<ContinuousExtruderMoveSequence>())
        {
            if (move_sequence->isClosed())
            {
                for (const std::shared_ptr<ExtrusionMove>& extrusion_move : move_sequence->getOperationsAs<ExtrusionMove>())
                {
                    feature_start_candidates.emplace_back(extrusion_move->getPosition(), feature_extrusion, move_sequence, extrusion_move, ChangeSequenceAction::Reorder);
                }
            }
            else
            {
                if (const std::optional<Point3LL> start_position = move_sequence->findStartPosition(); start_position.has_value())
                {
                    feature_start_candidates.emplace_back(start_position.value(), feature_extrusion, move_sequence, nullptr, ChangeSequenceAction::None);
                }

                if (const std::optional<Point3LL> end_position = move_sequence->findEndPosition(); end_position.has_value())
                {
                    feature_start_candidates.emplace_back(end_position.value(), feature_extrusion, move_sequence, nullptr, ChangeSequenceAction::Reverse);
                }
            }
        }

        start_candidates[feature_extrusion] = feature_start_candidates;
    }

    return start_candidates;
}

std::vector<FeatureExtrusionsOrderOptimizer::FeatureExtrusionOrderingConstraint>
    FeatureExtrusionsOrderOptimizer::makeFeatureExtrusionOrderingConstraints(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const
{
    std::vector<FeatureExtrusionOrderingConstraint> constraints;

    // Helper functions to iterate on features by type
    auto type_is = [](PrintFeatureType feature_type)
    {
        return [&feature_type](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
        {
            return feature_extrusion->getPrintFeatureType() == feature_type;
        };
    };

    auto type_is_not = [](PrintFeatureType feature_type)
    {
        return [&feature_type](const std::shared_ptr<FeatureExtrusion>& feature_extrusion)
        {
            return feature_extrusion->getPrintFeatureType() != feature_type;
        };
    };

    {
        // First process the bed adhesion features
        for (const std::shared_ptr<FeatureExtrusion>& adhesion_feature_extrusion : feature_extrusions | ranges::views::filter(type_is(PrintFeatureType::SkirtBrim)))
        {
            for (const std::shared_ptr<FeatureExtrusion>& non_adhesion_feature_extrusion : feature_extrusions | ranges::views::filter(type_is_not(PrintFeatureType::SkirtBrim)))
            {
                constraints.emplace_back(adhesion_feature_extrusion, non_adhesion_feature_extrusion);
            }
        }
    }

    return constraints;
}

std::vector<FeatureExtrusionsOrderOptimizer::MoveSequenceOrderingConstraint>
    FeatureExtrusionsOrderOptimizer::makeMoveSequenceOrderingConstraints(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const
{
    return {};
}

void FeatureExtrusionsOrderOptimizer::applyMoveSequenceAction(
    const std::shared_ptr<ContinuousExtruderMoveSequence>& move_sequence,
    const std::shared_ptr<ExtrusionMove>& move,
    const ChangeSequenceAction action)
{
    switch (action)
    {
    case ChangeSequenceAction::None:
        break;
    case ChangeSequenceAction::Reorder:
        if (move)
        {
            move_sequence->reorderToEndWith(move);
        }
        break;
    case ChangeSequenceAction::Reverse:
        move_sequence->reverse();
        break;
    }
}

Point3LL FeatureExtrusionsOrderOptimizer::optimizeExtruderSequencesOrder(
    const std::shared_ptr<FeatureExtrusion>& feature,
    const std::shared_ptr<ContinuousExtruderMoveSequence>& start_sequence,
    std::vector<StartCandidatePoint> start_candidates) const
{
    std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>> moves_sequences = feature->getOperationsAs<ContinuousExtruderMoveSequence>();
    std::vector<std::shared_ptr<PrintOperation>> ordered_sequences;
    ordered_sequences.reserve(moves_sequences.size());
    ordered_sequences.push_back(start_sequence);
    std::erase(moves_sequences, start_sequence);
    std::erase_if(
        start_candidates,
        [&start_sequence](const StartCandidatePoint& candidate_point)
        {
            return candidate_point.move_sequence == start_sequence;
        });

    std::optional<Point3LL> start_sequence_end_position = start_sequence->findEndPosition();
    assert(start_sequence_end_position.has_value() && "Unable to find the end position of the given start sequence");
    Point3LL current_position = start_sequence_end_position.value();

#warning This is extremely unoptimized, just good enough for the POC
    while (! moves_sequences.empty())
    {
        std::optional<ClosestPoint> closest_point;

        for (const StartCandidatePoint& start_candidate : start_candidates)
        {
            const coord_t distance_squared = (start_candidate.position - current_position).vSize2();
            if (! closest_point.has_value() || distance_squared < closest_point->distance_squared)
            {
                closest_point = ClosestPoint{ distance_squared, start_candidate };
            }
        }

        if (closest_point.has_value())
        {
            const StartCandidatePoint& best_candidate = closest_point->point;
            applyMoveSequenceAction(best_candidate.move_sequence, best_candidate.move, best_candidate.action);

            std::optional<Point3LL> end_position = best_candidate.move_sequence->findEndPosition().value();
            assert(end_position.has_value() && "The move sequence doesn't have an end position");
            current_position = end_position.value();
            ordered_sequences.push_back(best_candidate.move_sequence);
            std::erase(moves_sequences, best_candidate.move_sequence);

            std::erase_if(
                start_candidates,
                [&best_candidate](const StartCandidatePoint& candidate_point)
                {
                    return candidate_point.move_sequence == best_candidate.move_sequence;
                });
        }
        else
        {
            spdlog::error("If this happens, we are all gonna die quite soon, call your family and tell them you love them. Well, do it anyway, we are still gonna die someday, so "
                          "take care and enjoy.");
            break; // Try to save the world anyway, it's worth it
        }
    }

    feature->setOperations(ordered_sequences);

    return current_position;
}

} // namespace cura
