// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
#define PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H

#include <ranges>

#include "path_planning/ExtruderPlan.h"
#include "path_planning/FeatureExtrusionsPtr.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class ExtrusionMove;
class FeatureExtrusionsConstraintsGenerator;
struct FeatureExtrusionOrderingConstraint;

class FeatureExtrusionsOrderOptimizer final : public PrintOperationProcessor<ExtruderPlan>
{
public:
    explicit FeatureExtrusionsOrderOptimizer();

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
        StartCandidatePoint point;
    };

    struct MoveSequenceOrderingConstraint
    {
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_before;
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_after;
    };

    struct FeatureOrderingInformation
    {
        std::vector<StartCandidatePoint> start_candidates;
        std::vector<MoveSequenceOrderingConstraint> ordering_constraints;
    };

    using FeatureInformationMap = std::map<std::shared_ptr<FeatureExtrusion>, FeatureOrderingInformation>;

    using FeatureInformationKeysView = decltype(std::views::keys(std::declval<FeatureInformationMap>()));

private:
    std::map<std::shared_ptr<FeatureExtrusion>, std::vector<StartCandidatePoint>>
        makeStartCandidates(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const;

    std::vector<FeatureExtrusionOrderingConstraint> makeFeatureExtrusionOrderingConstraints(const std::vector<FeatureExtrusionPtr>& feature_extrusions) const;

    std::vector<MoveSequenceOrderingConstraint> makeMoveSequenceOrderingConstraints(const std::vector<std::shared_ptr<FeatureExtrusion>>& feature_extrusions) const;

    FeatureInformationMap makeFeatureOrderingInformation(const std::vector<FeatureExtrusionPtr>& feature_extrusions) const;

    static void applyMoveSequenceAction(
        const std::shared_ptr<ContinuousExtruderMoveSequence>& move_sequence,
        const std::shared_ptr<ExtrusionMove>& move,
        const ChangeSequenceAction action);

    void optimizeExtruderSequencesOrder(
        const std::shared_ptr<FeatureExtrusion>& feature,
        const std::shared_ptr<ContinuousExtruderMoveSequence>& start_sequence,
        std::vector<StartCandidatePoint> start_candidates);

    void findClosestPoint(const std::vector<StartCandidatePoint>& start_candidates, std::optional<ClosestPoint>& closest_point) const;

private:
#warning initialize with extruder start position
    Point3LL current_position_;
    std::vector<std::shared_ptr<FeatureExtrusionsConstraintsGenerator>> constraints_generators_;
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
