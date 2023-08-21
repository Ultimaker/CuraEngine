// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "pathPlanning/GCodePath.h"

namespace cura
{

[[nodiscard]] bool GCodePath::isTravelPath() const noexcept
{
    return config.isTravelPath();
}

[[nodiscard]] double GCodePath::getExtrusionMM3perMM() const noexcept
{
    return flow * width_factor * config.getExtrusionMM3perMM();
}

[[nodiscard]] coord_t GCodePath::getLineWidthForLayerView() const noexcept
{
    return static_cast<coord_t>(flow * width_factor * static_cast<double>(config.getLineWidth()) * config.getFlowRatio());
}

void GCodePath::setFanSpeed(const double fanspeed) noexcept
{
    fan_speed = fanspeed;
}

[[nodiscard]] double GCodePath::getFanSpeed() const noexcept
{
    return (fan_speed >= 0 && fan_speed <= 100) ? fan_speed : config.getFanSpeed();
}

} // namespace cura
