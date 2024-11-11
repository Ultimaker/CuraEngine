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
                const coord_t distance_squared = (start_candidate.position - current_position).vSize2();
                if (! closest_point.has_value() || distance_squared < closest_point->distance_squared)
                {
                    closest_point = ClosestPoint{ distance_squared, &start_candidate };
                }
            }
        }

        if (closest_point.has_value())
        {
            // Now apply the change to the move sequence to have it start with the given point
            const StartCandidatePoint* best_candidate = closest_point->point;

            switch (best_candidate->action)
            {
            case ChangeSequenceAction::None:
                break;
            case ChangeSequenceAction::Reorder:
                if (best_candidate->move)
                {
                    best_candidate->move_sequence->reorderToEndWith(best_candidate->move);
                }
                break;
            case ChangeSequenceAction::Reverse:
                best_candidate->move_sequence->reverse();
            }

            // Update current position for next candidate selection
            const std::optional<Point3LL> new_end_position = best_candidate->move_sequence->findEndPosition();
            assert(new_end_position.has_value() && "The new move sequence doesn't have an end position ?!");
            current_position = best_candidate->move_sequence->findEndPosition().value();

            // Register this feature as being processed next
            ordered_operations.push_back(best_candidate->feature_extrusion);

            // Remove processed feature from processing list
            std::erase(feature_extrusions, best_candidate->feature_extrusion);

            // Remove constraints related to this feature
            std::erase_if(
                feature_extrusions_constraints,
                [&best_candidate](const FeatureExtrusionOrderingConstraint& constraint)
                {
                    return constraint.feature_before == best_candidate->feature_extrusion;
                });
            ;
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

} // namespace cura
