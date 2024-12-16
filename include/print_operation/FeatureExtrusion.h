// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_FEATUREEXTRUSION_H
#define PATHPLANNING_FEATUREEXTRUSION_H

#include "GCodePathConfig.h"
#include "print_operation/ContinuousExtruderMoveSequencePtr.h"
#include "print_operation/FeatureExtrusionPtr.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class Shape;

class FeatureExtrusion : public PrintOperationSequence
{
public:
    explicit FeatureExtrusion(const PrintFeatureType type, const coord_t nominal_line_width);

    void appendExtruderMoveSequence(const ContinuousExtruderMoveSequencePtr& extruder_move_sequence, bool check_non_empty = true);

    PrintFeatureType getType() const;

    coord_t getNominalLineWidth() const;

private:
    const PrintFeatureType type_;
    const coord_t nominal_line_width_;
};

} // namespace cura

#endif // PATHPLANNING_FEATUREEXTRUSION_H
