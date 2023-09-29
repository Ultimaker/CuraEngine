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

    constexpr bool operator==(const SpeedDerivatives& other) const noexcept = default;
    constexpr auto operator<=>(const SpeedDerivatives& other) const noexcept = default;

    /*!
     * Set the speed to somewhere between the speed of @p first_layer_config and the iconic speed.
     *
     * \warning This functions should not be called with @p layer_nr > @p max_speed_layer !
     *
     * \warning Calling this function twice will smooth the speed more toward \p first_layer_config
     *
     * \param first_layer_config The speed settings at layer zero
     * \param layer_nr The layer number
     * \param max_speed_layer The layer number for which the speed_iconic should be used.
     */
    void smoothSpeed(const SpeedDerivatives& first_layer_config, const LayerIndex layer_nr, const LayerIndex max_speed_layer_nr);
};

} // namespace cura

#endif // PATHPLANNING_SPEEDDERIVATIVES_H
