// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "pathPlanning/SpeedDerivatives.h"

namespace cura
{

void SpeedDerivatives::smoothSpeed(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer_nr)
{
    const auto max_speed_layer = static_cast<double>(max_speed_layer_nr);
    const auto first_layer_speed = std::min(speed, first_layer_config.speed);
    const auto first_layer_acceleration = std::min(acceleration, first_layer_config.acceleration);
    const auto first_layer_jerk = std::min(jerk, first_layer_config.jerk);
    speed = (speed * static_cast<double>(layer_nr)) / max_speed_layer + (first_layer_speed * (max_speed_layer - static_cast<double>(layer_nr)) / max_speed_layer);
    acceleration
        = (acceleration * static_cast<double>(layer_nr)) / max_speed_layer + (first_layer_acceleration * (max_speed_layer - static_cast<double>(layer_nr)) / max_speed_layer);
    jerk = (jerk * static_cast<double>(layer_nr)) / max_speed_layer + (first_layer_jerk * (max_speed_layer - static_cast<double>(layer_nr)) / max_speed_layer);
}

} // namespace cura