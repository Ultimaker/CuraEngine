// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
#define PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H

#include <map>

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
    using SequencesConstraintsMap = std::map<std::shared_ptr<ContinuousExtruderMoveSequence>, std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>>>;

private:
    static std::vector<FeatureExtrusionPtr> makeOrderingContraints(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions);

    SequencesConstraintsMap makeMoveSequencesConstraints(const FeatureExtrusionPtr& feature_extrusion);

    static std::shared_ptr<ZSeamConfig> getZSeamConfig(const FeatureExtrusionPtr& feature_extrusion);

    void makeStartCandidates(const FeatureExtrusionPtr& feature_extrusion);

    static void applyMoveSequenceAction(const StartCandidatePoint& start_point);

    bool moveSequenceProcessableNow(const std::shared_ptr<ContinuousExtruderMoveSequence>& move_sequence) const;

    void appendNextProcessedSequence(
        const StartCandidatePoint& start_point,
        std::vector<std::shared_ptr<PrintOperation>>& ordered_sequences,
        std::vector<std::shared_ptr<ContinuousExtruderMoveSequence>>& moves_sequences,
        Point3LL& current_position);

private:
    const std::shared_ptr<ZSeamConfig> seam_config_;
    const std::vector<FeatureExtrusionPtr> extrusions_after_;
    SequencesConstraintsMap sequences_constraints_;
    std::vector<StartCandidatePoint> start_candidates_;
    const std::shared_ptr<Shape> overhang_areas_;
    bool optimize_extrusion_sequences_{ true };
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
