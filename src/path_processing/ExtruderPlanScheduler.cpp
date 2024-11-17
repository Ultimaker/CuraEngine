// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/ExtruderPlanScheduler.h"

#include <spdlog/spdlog.h>

#include "path_planning/FeatureExtrusion.h"
#include "path_processing/ClosestStartPoint.h"
#include "path_processing/FeatureExtrusionScheduler.h"

namespace cura
{

void ExtruderPlanScheduler::process(ExtruderPlan* extruder_plan)
{
    std::map<FeatureExtrusionPtr, std::shared_ptr<FeatureExtrusionScheduler>> feature_extrusion_schedulers;

    { // Make individual scheduler for each feature extrusion
        std::vector<std::shared_ptr<FeatureExtrusion>> feature_extrusions = extruder_plan->getOperationsAs<FeatureExtrusion>();
        for (const std::shared_ptr<FeatureExtrusion>& feature_extrusion : feature_extrusions)
        {
            feature_extrusion_schedulers[feature_extrusion] = std::make_shared<FeatureExtrusionScheduler>(feature_extrusion, feature_extrusions);
        }
    }

    // Make a view containing the features for easier iteration
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
            // Optimizer inner sequences of feature, starting by the sequence holding the best candidate, and update current position for next candidate selection
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
            spdlog::error("Unable to find a start candidates amongst extrusion feature, some constraints must be contradictory to each other");
            break;
        }
    }

    extruder_plan->setOperations(ordered_operations);
}

} // namespace cura
