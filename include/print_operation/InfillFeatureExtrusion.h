// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "infill.h"
#include "print_operation/MeshFeatureExtrusion.h"
#include "settings/types/Angle.h"

namespace cura
{

class InfillFeatureExtrusion : public MeshFeatureExtrusion
{
public:
    explicit InfillFeatureExtrusion(
        const PrintFeatureType type,
        const coord_t nominal_line_width,
        const std::shared_ptr<const SliceMeshStorage>& mesh,
        const AngleDegrees& infill_angle);

    static std::shared_ptr<InfillFeatureExtrusion> makeFrom(
        const Infill::GeneratedPatterns& patterns,
        PrintFeatureType type,
        const coord_t line_width,
        const std::shared_ptr<const SliceMeshStorage>& mesh,
        const AngleDegrees& infill_angle,
        const Velocity& speed);

    const AngleDegrees& GetInfillAngle() const;

private:
    AngleDegrees infill_angle_;
};

} // namespace cura
