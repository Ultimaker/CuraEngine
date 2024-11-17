// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_STARTCANDIDATEPOINT_H
#define PATHPROCESSING_STARTCANDIDATEPOINT_H

#include <geometry/Point3LL.h>

#include "path_planning/FeatureExtrusionPtr.h"
#include "path_processing/ChangeSequenceAction.h"

namespace cura
{

class ContinuousExtrusionMoveSequence;
class ExtrusionMove;

struct StartCandidatePoint
{
    Point3LL position;
#warning This value may be useless now, check
    FeatureExtrusionPtr feature_extrusion;
    std::shared_ptr<ContinuousExtrusionMoveSequence> move_sequence;
    std::shared_ptr<ExtrusionMove> move; // The move containing the target position, or null for the actual starting point
    ChangeSequenceAction action; // The action to be applied if starting by this point
};

} // namespace cura

#endif // PATHPROCESSING_STARTCANDIDATEPOINT_H
