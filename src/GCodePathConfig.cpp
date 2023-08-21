// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GCodePathConfig.h"

#include "utils/IntPoint.h" // INT2MM

namespace cura
{

[[nodiscard]] double GCodePathConfig::getExtrusionMM3perMM() const noexcept
{
    return extrusion_mm3_per_mm;
}

[[nodiscard]] Velocity GCodePathConfig::getSpeed() const noexcept
{
    return speed_derivatives.speed;
}

[[nodiscard]] Acceleration GCodePathConfig::getAcceleration() const noexcept
{
    return speed_derivatives.acceleration;
}

[[nodiscard]] Velocity GCodePathConfig::getJerk() const noexcept
{
    return speed_derivatives.jerk;
}

[[nodiscard]] coord_t GCodePathConfig::getLineWidth() const noexcept
{
    return line_width;
}

[[nodiscard]] coord_t GCodePathConfig::getLayerThickness() const noexcept
{
    return layer_thickness;
}

[[nodiscard]] PrintFeatureType GCodePathConfig::getPrintFeatureType() const noexcept
{
    return type;
}

[[nodiscard]] bool GCodePathConfig::isTravelPath() const noexcept
{
    return line_width == 0;
}

[[nodiscard]] bool GCodePathConfig::isBridgePath() const noexcept
{
    return is_bridge_path;
}

[[nodiscard]] double GCodePathConfig::getFanSpeed() const noexcept
{
    return fan_speed;
}

[[nodiscard]] Ratio GCodePathConfig::getFlowRatio() const noexcept
{
    return flow;
}

[[nodiscard]] double GCodePathConfig::calculateExtrusion() const noexcept
{
    return INT2MM(line_width) * INT2MM(layer_thickness) * double(flow);
}


} // namespace cura
