// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_MOVESEQUENCESCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_MOVESEQUENCESCONSTRAINTSGENERATOR_H

#include <map>
#include <vector>

#include "path_planning/ContinuousExtruderMoveSequencePtr.h"
#include "path_planning/FeatureExtrusionPtr.h"

namespace cura
{

class MoveSequencesConstraintsGenerator
{
public:
    using SequencesConstraintsMap = std::map<ContinuousExtruderMoveSequencePtr, std::vector<ContinuousExtruderMoveSequencePtr>>;

public:
    virtual void appendConstraints(const FeatureExtrusionPtr& feature_extrusion, SequencesConstraintsMap& constraints) const = 0;
};

} // namespace cura

#endif // PATHPROCESSING_MOVESEQUENCESCONSTRAINTSGENERATOR_H
