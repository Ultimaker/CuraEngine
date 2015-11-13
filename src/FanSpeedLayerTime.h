#ifndef FAN_SPEED_LAYER_TIME_H
#define FAN_SPEED_LAYER_TIME_H

#include "settings.h"

namespace cura 
{

struct FanSpeedLayerTimeSettings 
{
public:
    double cool_min_layer_time;
    double cool_min_layer_time_fan_speed_max;
    double cool_fan_speed_min;
    double cool_fan_speed_max;
    double cool_min_speed;
    int cool_fan_full_layer;
};

} // namespace cura

#endif // FAN_SPEED_LAYER_TIME_H