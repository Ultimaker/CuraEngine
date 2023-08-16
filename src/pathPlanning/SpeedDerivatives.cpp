// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "pathPlanning/SpeedDerivatives.h"

namespace cura
{

constexpr void SpeedDerivatives::smoothSpeed(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer_nr)
{
    double max_speed_layer = max_speed_layer_nr;
    double first_layer_speed = std::min(speed, first_layer_config.speed);
    double first_layer_acceleration = std::min(acceleration, first_layer_config.acceleration);
    double first_layer_jerk = std::min(jerk, first_layer_config.jerk);
    speed = (speed * layer_nr) / max_speed_layer + (first_layer_speed * (max_speed_layer - layer_nr) / max_speed_layer);
    acceleration = (acceleration * layer_nr) / max_speed_layer + (first_layer_acceleration * (max_speed_layer - layer_nr) / max_speed_layer);
    jerk = (jerk * layer_nr) / max_speed_layer + (first_layer_jerk * (max_speed_layer - layer_nr) / max_speed_layer);
}

} // namespace cura