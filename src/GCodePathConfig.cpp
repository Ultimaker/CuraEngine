// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GCodePathConfig.h"

#include "geometry/Point2LL.h" // INT2MM
#include "types/Acceleration.h"

namespace curaengine
{

[[nodiscard]] double GCodePathConfig::getExtrusionMM3perMM() const noexcept
{
    return extrusion_mm3_per_mm;
}

[[nodiscard]] ::curaengine::types::Velocity GCodePathConfig::getSpeed() const noexcept
{
    return speed_derivatives.speed;
}

[[nodiscard]] ::curaengine::types::Acceleration GCodePathConfig::getAcceleration() const noexcept
{
    return speed_derivatives.acceleration;
}

[[nodiscard]] ::curaengine::types::Velocity GCodePathConfig::getJerk() const noexcept
{
    return speed_derivatives.jerk;
}

[[nodiscard]] ::curaengine::utils::coord_t GCodePathConfig::getLineWidth() const noexcept
{
    return line_width;
}

[[nodiscard]] ::curaengine::utils::coord_t GCodePathConfig::getLayerThickness() const noexcept
{
    return layer_thickness;
}

[[nodiscard]] PrintFeatureType GCodePathConfig::getPrintFeatureType() const noexcept
{
    return type;
}

const ::curaengine::types::Temperature& GCodePathConfig::getTemperatureDelta() const noexcept
{
    return temperature_delta;
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

[[nodiscard]] ::curaengine::types::Ratio GCodePathConfig::getFlowRatio() const noexcept
{
    return flow;
}

[[nodiscard]] double GCodePathConfig::calculateExtrusion() const noexcept
{
    return INT2MM(line_width) * INT2MM(layer_thickness) * double(flow);
}


} // namespace curaengine
