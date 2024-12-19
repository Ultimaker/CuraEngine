// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/InfillFeatureExtrusion.h"

namespace cura
{

InfillFeatureExtrusion::InfillFeatureExtrusion(const PrintFeatureType type, const coord_t nominal_line_width, const std::shared_ptr<const SliceMeshStorage>& mesh, const AngleDegrees &infill_angle)
    : MeshFeatureExtrusion(type, nominal_line_width, mesh), infill_angle_(infill_angle)
{
}

const AngleDegrees& InfillFeatureExtrusion::GetInfillAngle() const
{
    return infill_angle_;
}

} // namespace cura
