// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/InfillFeatureExtrusion.h"

#include "geometry/OpenPolyline.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"

namespace cura
{

InfillFeatureExtrusion::InfillFeatureExtrusion(
    const PrintFeatureType type,
    const coord_t nominal_line_width,
    const std::shared_ptr<const SliceMeshStorage>& mesh,
    const AngleDegrees& infill_angle)
    : MeshFeatureExtrusion(type, nominal_line_width, mesh)
    , infill_angle_(infill_angle)
{
}

std::shared_ptr<InfillFeatureExtrusion> InfillFeatureExtrusion::makeFrom(
    const Infill::GeneratedPatterns& patterns,
    PrintFeatureType type,
    const coord_t line_width,
    const std::shared_ptr<const SliceMeshStorage>& mesh,
    const AngleDegrees& infill_angle,
    const Velocity& speed)
{
    auto feature_extrusion = std::make_shared<InfillFeatureExtrusion>(type, line_width, mesh, infill_angle);

    for (const Polygon& polygon : patterns.polygons)
    {
        feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(polygon, line_width, speed));
    }

    for (const OpenPolyline& polyline : patterns.lines)
    {
        feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(polyline, line_width, speed));
    }

    for (const VariableWidthLines& extrusion_lines : patterns.toolpaths)
    {
        for (const ExtrusionLine& extrusion_line : extrusion_lines)
        {
            feature_extrusion->appendExtruderMoveSequence(ContinuousExtruderMoveSequence::makeFrom(extrusion_line, speed));
        }
    }

    return feature_extrusion;
}

const AngleDegrees& InfillFeatureExtrusion::GetInfillAngle() const
{
    return infill_angle_;
}

} // namespace cura
