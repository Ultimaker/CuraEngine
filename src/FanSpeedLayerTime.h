//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FAN_SPEED_LAYER_TIME_H
#define FAN_SPEED_LAYER_TIME_H

#include "settings/types/Duration.h"
#include "settings/types/LayerIndex.h"
#include "settings/types/Velocity.h"

namespace cura 
{

/*!
 * A small struct to collate settings related to fan speed and cooling.
 *
 * This is similar to the PathConfig struct in its function. You can pass just
 * a single one around and make it apply to all parts, rather than having to
 * store these settings over and over again for each part, even though the
 * settings may be different for each part on a layer.
 */
struct FanSpeedLayerTimeSettings 
{
public:
    /*!
     * Minimum layer time. How long should the layer get to cool down before the
     * next layer is allowed to begin printing on it?
     */
    Duration cool_min_layer_time;

    /*!
     * "Regular/Maximum Fan Speed Threshold". If the layers take longer to print
     * than this, they'll use the regular fan speed. If they take shorter, we'll
     * interpolate between regular and maximum fan speed.
     */
    Duration cool_min_layer_time_fan_speed_max;

    /*!
     * Fan speed on the initial layer.
     */
    double cool_fan_speed_0;

    /*!
     * Regular fan speed, used for most of the print, during layers longer than
     * the Regular/Maximum Fan Speed Threshold.
     */
    double cool_fan_speed_min;

    /*!
     * "Maximum Fan Speed", the fan speed when the minimum layer time is
     * reached. As the layers get shorter and you're approaching the minimum
     * layer time, the fan speed is linearly interpolated between the Minimum
     * Layer Time and the Regular/Maximum Fan Speed Threshold, to approach the
     * maximum fan speed.
     */
    double cool_fan_speed_max;

    /*!
     * Minimum Speed. We'll slow down the print speed if we're at the minimum
     * layer time, but only up to a point. If it needs to be slowed down too
     * much, it would degrade the filament. So don't slow down below this value.
     */
    Velocity cool_min_speed;

    /*!
     * For the initial layer fan speed we'll gradually increase the fan speed to
     * the regular fan speed across a number of layers. This is that number of
     * layers.
     */
    LayerIndex cool_fan_full_layer;
};

} // namespace cura

#endif // FAN_SPEED_LAYER_TIME_H