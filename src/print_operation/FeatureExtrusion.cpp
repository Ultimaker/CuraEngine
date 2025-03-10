// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/FeatureExtrusion.h"

#include <geometry/MixedLinesSet.h>
#include <geometry/Shape.h>

#include <range/v3/numeric/accumulate.hpp>

#include "print_operation/ContinuousExtruderMoveSequence.h"
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

Shape FeatureExtrusion::calculateFootprint() const
{
    // Do not consider individual lines widths because clipper is not able to make a multi-width offset. The result
    // is then very approximate but should be good enough in most cases. If not, then this behavior should be improved
    MixedLinesSet extrusions_polylines;

    for (const ContinuousExtruderMoveSequencePtr& extruder_move_sequence : getOperationsAs<ContinuousExtruderMoveSequence>())
    {
        extrusions_polylines.push_back(extruder_move_sequence->calculatePolyline());
    }

    return extrusions_polylines.offset(nominal_line_width_ / 2);
}

coord_t FeatureExtrusion::calculateLength() const
{
    return ranges::accumulate(getOperations(), 0, [](coord_t length, const PrintOperationPtr &operation) -> coord_t
    {
        if (auto move_sequence = std::dynamic_pointer_cast<ContinuousExtruderMoveSequence>(operation))
        {
            return length + move_sequence->calculateLength();
        }
        return length;
    });
}

} // namespace cura
