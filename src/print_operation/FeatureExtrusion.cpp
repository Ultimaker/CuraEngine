// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/FeatureExtrusion.h"

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/ExtrusionMove.h"

namespace cura
{

FeatureExtrusion::FeatureExtrusion(const PrintFeatureType type, const coord_t nominal_line_width)
    : type_(type)
    , nominal_line_width_(nominal_line_width)
{
}

void FeatureExtrusion::appendExtruderMoveSequence(const ContinuousExtruderMoveSequencePtr& extruder_move_sequence, bool check_non_empty)
{
    if (! check_non_empty || ! extruder_move_sequence->empty())
    {
        appendOperation(extruder_move_sequence);
    }
}

PrintFeatureType FeatureExtrusion::getType() const
{
    return type_;
}
coord_t FeatureExtrusion::getNominalLineWidth() const
{
    return nominal_line_width_;
}

} // namespace cura
