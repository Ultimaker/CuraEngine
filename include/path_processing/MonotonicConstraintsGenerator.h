// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H

#include <utils/Coord_t.h>

#include "path_processing/MoveSequencesConstraintsGenerator.h"

namespace cura
{

class AngleDegrees;

class MonotonicConstraintsGenerator : public MoveSequencesConstraintsGenerator
{
public:
    void appendConstraints(const FeatureExtrusionPtr& feature_extrusion, SequencesConstraintsMap& constraints) const override;

private:
    void appendMonotonicConstraints(
        const std::vector<ContinuousExtruderMoveSequencePtr>& moves,
        const AngleDegrees& angle,
        const coord_t same_line_distance,
        SequencesConstraintsMap& constraints) const;
};

} // namespace cura

#endif // PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H
