// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
#define PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H

#include <map>
#include <optional>

#include "operation_transformation/StartCandidatePoint.h"
#include "print_operation/ContinuousExtruderMoveSequencePtr.h"
#include "print_operation/FeatureExtrusionPtr.h"

namespace cura
{

class Shape;
class PrintOperation;
class MoveSequencesConstraintsGenerator;
struct ZSeamConfig;
struct ClosestStartPoint;

class FeatureExtrusionScheduler
{
public:
    explicit FeatureExtrusionScheduler(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions);

    bool isFeatureProcessableNow(const FeatureExtrusionPtr& feature_extrusion) const;

    void evaluateClosestPoint(std::optional<ClosestStartPoint>& closest_point, const Point3LL& last_position) const;

    void optimizeExtruderSequencesOrder(const StartCandidatePoint& start_point, Point3LL& current_position);

private:
    using SequencesConstraintsMap = std::map<ContinuousExtruderMoveSequencePtr, std::vector<ContinuousExtruderMoveSequencePtr>>;

    using StartCandidatesBySequenceMap = std::map<ContinuousExtruderMoveSequencePtr, std::vector<StartCandidatePoint>>;

private:
    static std::vector<FeatureExtrusionPtr> makeOrderingConstraints(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions);

    SequencesConstraintsMap makeMoveSequencesConstraints(const FeatureExtrusionPtr& feature_extrusion);

    static std::shared_ptr<ZSeamConfig> getZSeamConfig(const FeatureExtrusionPtr& feature_extrusion);

    void makeStartCandidates(const FeatureExtrusionPtr& feature_extrusion);

    static void applyMoveSequenceAction(const StartCandidatePoint& start_point);

    bool moveSequenceProcessableNow(const ContinuousExtruderMoveSequencePtr& move_sequence) const;

    void appendNextProcessedSequence(
        const StartCandidatePoint& start_point,
        std::vector<std::shared_ptr<PrintOperation>>& ordered_sequences,
        std::vector<ContinuousExtruderMoveSequencePtr>& moves_sequences,
        Point3LL& current_position);

    static std::vector<StartCandidatePoint> makeBaseStartCandidates(const FeatureExtrusionPtr& feature, const ContinuousExtruderMoveSequencePtr& move_sequence);

    void preFilterStartCandidates(std::vector<StartCandidatePoint>& start_candidates);

private:
    const std::shared_ptr<ZSeamConfig> seam_config_;
    const std::vector<FeatureExtrusionPtr> extrusions_after_;
    SequencesConstraintsMap moves_constraints_;
    StartCandidatesBySequenceMap start_candidates_;
    const std::shared_ptr<Shape> overhang_areas_;
    bool optimize_extrusion_sequences_{ true };
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONCHEDULER_H
