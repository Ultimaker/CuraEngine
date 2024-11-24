// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H
#define PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H

#include <utils/Coord_t.h>

#include "path_processing/MoveSequencesConstraintsGenerator.h"

namespace cura
{

class AngleDegrees;
class Point2D;

class MonotonicConstraintsGenerator : public MoveSequencesConstraintsGenerator
{
public:
    void appendConstraints(const FeatureExtrusionPtr& feature_extrusion, SequencesConstraintsMap& constraints) const override;

private:
    static void appendMonotonicConstraints(
        const std::vector<ContinuousExtruderMoveSequencePtr>& moves,
        const AngleDegrees& angle,
        const double same_line_distance,
        const double max_adjacent_distance,
        SequencesConstraintsMap& constraints);
};

} // namespace cura

#endif // PATHPROCESSING_MONOTONICCONSTRAINTSGENERATOR_H
