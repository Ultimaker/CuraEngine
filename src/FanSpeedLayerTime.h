//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FAN_SPEED_LAYER_TIME_H
#define FAN_SPEED_LAYER_TIME_H

#include "settings/types/Duration.h"
#include "settings/types/LayerIndex.h"
#include "settings/types/Velocity.h"

namespace cura 
{

struct FanSpeedLayerTimeSettings 
{
public:
    Duration cool_min_layer_time;
    Duration cool_min_layer_time_fan_speed_max;
    double cool_fan_speed_0;
    double cool_fan_speed_min;
    double cool_fan_speed_max;
    Velocity cool_min_speed;
    LayerIndex cool_fan_full_layer;
};

} // namespace cura

#endif // FAN_SPEED_LAYER_TIME_H