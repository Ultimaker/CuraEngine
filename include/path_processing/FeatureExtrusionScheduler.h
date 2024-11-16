// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
#define PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H

#include "path_planning/FeatureExtrusion.h"
#include "path_planning/FeatureExtrusionsPtr.h"
#include "path_processing/StartCandidatePoint.h"

namespace cura
{

class ExtrusionMove;
class FeatureExtrusionsConstraintsGenerator;
class Shape;
struct FeatureExtrusionOrderingConstraint;
struct ZSeamConfig;
struct ClosestStartPoint;

class FeatureExtrusionScheduler final
{
public:
    explicit FeatureExtrusionScheduler(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions);

    bool isFeatureProcessableNow(const FeatureExtrusionPtr& feature_extrusion) const;

    void evaluateClosestPoint(std::optional<ClosestStartPoint>& closest_point, const Point3LL& last_position) const;

    void optimizeExtruderSequencesOrder(const StartCandidatePoint& start_point, Point3LL& current_position);

private:
    struct MoveSequenceOrderingConstraint
    {
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_before;
        std::shared_ptr<ContinuousExtruderMoveSequence> sequence_after;
    };

private:
    static std::vector<FeatureExtrusionPtr> makeOrderingContraints(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions);

    static std::vector<MoveSequenceOrderingConstraint> makeMoveSequencesConstraints(const FeatureExtrusionPtr& feature_extrusion);

    static std::shared_ptr<ZSeamConfig> getZSeamConfig(const FeatureExtrusionPtr& feature_extrusion);

    void makeStartCandidates(const FeatureExtrusionPtr& feature_extrusion);

    static void applyMoveSequenceAction(const StartCandidatePoint& start_point);

private:
    const std::shared_ptr<ZSeamConfig> seam_config_;
    const std::vector<FeatureExtrusionPtr> extrusions_after_;
    const std::vector<MoveSequenceOrderingConstraint> sequences_constraints_;
    std::vector<StartCandidatePoint> start_candidates_;
    const std::shared_ptr<Shape> overhang_areas_;
    bool optimize_extrusion_sequences_{ true };
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
