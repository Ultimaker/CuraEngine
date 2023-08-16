// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_SPEEDDERIVATIVES_H
#define PATHPLANNING_SPEEDDERIVATIVES_H

#include "settings/types/LayerIndex.h"
#include "settings/types/Velocity.h"

namespace cura
{

struct SpeedDerivatives
{
    Velocity speed{}; //!< movement speed (mm/s)
    Acceleration acceleration{}; //!< acceleration of head movement (mm/s^2)
    Velocity jerk{}; //!< jerk of the head movement (around stand still) as instantaneous speed change (mm/s)

    constexpr void smoothSpeed(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer_nr);
};

} // namespace cura

#endif // PATHPLANNING_SPEEDDERIVATIVES_H
