// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
#define PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H

#include "path_planning/ExtruderPlan.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class ExtrusionMove;

class FeatureExtrusionsOrderOptimizer final : public PrintOperationProcessor<ExtruderPlan>
{
public:
    explicit FeatureExtrusionsOrderOptimizer(const Point3LL& previous_position);

    void process(ExtruderPlan* extruder_plan) override;

private:
    enum class ChangeSequenceAction
    {
        None, // Nothing to do, point is already the start point
        Reverse, // Reverse the (open) extrusion sequence, point is the last one
        Reorder // Reorder the (closed) extrusion sequence so that it starts/ends with the point
    };

    struct StartCandidatePoint
    {
        Point3LL position;
        std::shared_ptr<FeatureExtrusion> feature_extrusion;
        std::shared_ptr<ContinuousExtruderMoveSequence> move_sequence;
        std::shared_ptr<ExtrusionMove> move; // The move containing the target position, or null for the actual starting point
        ChangeSequenceAction action; // The action to be applied if starting by this point
    };

    struct ClosestPoint
    {
        coord_t distance_squared;
        const StartCandidatePoint* point;
    };

    struct FeatureExtrusionOrderingConstraint
    {
        std::shared_ptr<FeatureExtrusion> feature_before;
        std::shared_ptr<FeatureExtrusion> feature_after;
    };

    struct MoveSequenceOrderingConstraint
    {
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_before;
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_after;
    };

private:
    std::map<std::shared_ptr<FeatureExtrusion>, std::vector<StartCandidatePoint>>
        makeStartCandidates(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const;

    std::vector<FeatureExtrusionOrderingConstraint> makeFeatureExtrusionOrderingConstraints(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const;

    std::vector<MoveSequenceOrderingConstraint> makeMoveSequenceOrderingConstraints(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const;

private:
    const Point3LL previous_position_;
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
