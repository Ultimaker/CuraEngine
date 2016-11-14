#include "Preheat.h"

namespace cura 
{

double Preheat::getTimeToGoFromTempToTemp(int extruder, double temp_before, double temp_after, bool during_printing)
{
    Config& config = config_per_extruder[extruder];
    if (temp_after > temp_before)
    {
        return (temp_after - temp_before) * (config.time_to_heatup_1_degree + during_printing * config.heatup_cooldown_time_mod_while_printing);
    }
    else
    {
        return (temp_before - temp_after) * (config.time_to_cooldown_1_degree - during_printing * config.heatup_cooldown_time_mod_while_printing);
    }
}

Preheat::CoolDownResult Preheat::timeBeforeEndToInsertPreheatCommand_warmUpCoolDown(double time_window, unsigned int extruder, double temp0, double temp1, double temp2, bool during_printing)
{
    CoolDownResult result;
    const Config& config = config_per_extruder[extruder];
    result.total_time_window = time_window;

    //      limited_time_window
    //      ^^^^^^^^^^
    //               _> temp1
    //       /""""""\                                       .
    //      / . . . .\ . . .> outer_temp                    .
    //     ^temp0     \                                     .
    //                 \                                    .
    //                  ^temp2
    double outer_temp;
    double limited_time_window;
    double extra_cooldown_time = 0;
    if (temp0 > temp2)
    { // extra time needed during heating
        double extra_heatup_time = (temp0 - temp2) * (config.time_to_heatup_1_degree + during_printing * config.heatup_cooldown_time_mod_while_printing);
        limited_time_window = time_window - extra_heatup_time;
        outer_temp = temp0;
    }
    else
    {
        extra_cooldown_time = (temp2 - temp0) * (config.time_to_cooldown_1_degree - during_printing * config.heatup_cooldown_time_mod_while_printing);
        limited_time_window = time_window - extra_cooldown_time;
        outer_temp = temp2;
    }
    double time_ratio_heatup_cooldown = config.time_to_heatup_1_degree / config.time_to_cooldown_1_degree;
    double cool_down_time = getTimeToGoFromTempToTemp(extruder, outer_temp, temp1, during_printing);
    double time_needed_to_reach_temp1 = cool_down_time * (1.0 + time_ratio_heatup_cooldown);
    if (time_needed_to_reach_temp1 < limited_time_window)
    {
        result.cooling_time = cool_down_time;
        result.highest_temperature = temp1;
    }
    else 
    {
        result.cooling_time = limited_time_window * config.time_to_heatup_1_degree / (config.time_to_cooldown_1_degree + config.time_to_heatup_1_degree);
        result.highest_temperature = std::min(temp1, temp2 + result.cooling_time / config.time_to_cooldown_1_degree);
    }

    result.cooling_time += extra_cooldown_time;
    return result;
}


}//namespace cura
