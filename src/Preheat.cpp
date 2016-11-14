#include "Preheat.h"

namespace cura 
{

void Preheat::setConfig(MeshGroup& settings)
{
    for (int extruder_nr = 0; extruder_nr < settings.getExtruderCount(); extruder_nr++)
    {
        assert(settings.getExtruderTrain(extruder_nr) != nullptr);
        ExtruderTrain& extruder_train = *settings.getExtruderTrain(extruder_nr);
        config_per_extruder.emplace_back();
        Config& config = config_per_extruder.back();
        config.time_to_cooldown_1_degree = 1.0 / extruder_train.getSettingInSeconds("machine_nozzle_cool_down_speed"); // 0.5
        config.time_to_heatup_1_degree = 1.0 / extruder_train.getSettingInSeconds("machine_nozzle_heat_up_speed"); // 0.5
        config.heatup_cooldown_time_mod_while_printing = 1.0 / extruder_train.getSettingInSeconds("material_extrusion_cool_down_speed"); // 0.1
        config.standby_temp = extruder_train.getSettingInSeconds("material_standby_temperature"); // 150

        config.min_time_window = extruder_train.getSettingInSeconds("machine_min_cool_heat_time_window");

        config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature");
        config.material_print_temperature_layer_0 = extruder_train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        config.material_initial_print_temperature = extruder_train.getSettingInDegreeCelsius("material_initial_print_temperature");
        config.material_final_print_temperature = extruder_train.getSettingInDegreeCelsius("material_final_print_temperature");

        config.flow_dependent_temperature = extruder_train.getSettingBoolean("material_flow_dependent_temperature"); 

        config.flow_temp_graph = extruder_train.getSettingAsFlowTempGraph("material_flow_temp_graph"); // [[0.1,180],[20,230]]
    }
}

double Preheat::timeToHeatFromStandbyToPrintTemp(unsigned int extruder, double temp)
{
    return (temp - config_per_extruder[extruder].standby_temp) * config_per_extruder[extruder].time_to_heatup_1_degree; 
}

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

double Preheat::getTemp(unsigned int extruder, double flow, bool is_initial_layer)
{
    if (is_initial_layer)
    {
        return config_per_extruder[extruder].material_print_temperature_layer_0;
    }
    return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature, config_per_extruder[extruder].flow_dependent_temperature);
}

Preheat::WarmUpResult Preheat::timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(double time_window, unsigned int extruder, double temp)
{
    WarmUpResult result;
    const Config& config = config_per_extruder[extruder];
    result.total_time_window = time_window;
    double time_ratio_cooldown_heatup = config.time_to_cooldown_1_degree / config.time_to_heatup_1_degree;
    double time_to_heat_from_standby_to_print_temp = timeToHeatFromStandbyToPrintTemp(extruder, temp);
    double time_needed_to_reach_standby_temp = time_to_heat_from_standby_to_print_temp * (1.0 + time_ratio_cooldown_heatup);
    if (time_needed_to_reach_standby_temp < time_window)
    {
        result.heating_time = time_to_heat_from_standby_to_print_temp;
        result.lowest_temperature = config.standby_temp;
    }
    else 
    {
        result.heating_time = time_window * config.time_to_heatup_1_degree / (config.time_to_cooldown_1_degree + config.time_to_heatup_1_degree);
        result.lowest_temperature = std::max(config.standby_temp, temp - result.heating_time / config.time_to_heatup_1_degree);
    }
    return result;
}

Preheat::CoolDownResult Preheat::timeBeforeEndToInsertPreheatCommand_warmUpCoolDown(double time_window, unsigned int extruder, double temp0, double temp1, double temp2, bool during_printing)
{
    CoolDownResult result;
    const Config& config = config_per_extruder[extruder];
    double time_to_cooldown_1_degree = config.time_to_cooldown_1_degree - during_printing * config.heatup_cooldown_time_mod_while_printing;
    double time_to_heatup_1_degree = config.time_to_heatup_1_degree + during_printing * config.heatup_cooldown_time_mod_while_printing;

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
    if (temp0 < temp2)
    { // extra time needed during heating
        double extra_heatup_time = (temp2 - temp0) * time_to_heatup_1_degree;
        limited_time_window = time_window - extra_heatup_time;
        outer_temp = temp0;
    }
    else
    {
        extra_cooldown_time = (temp0 - temp2) * time_to_cooldown_1_degree;
        limited_time_window = time_window - extra_cooldown_time;
        outer_temp = temp2;
    }
    double time_ratio_cooldown_heatup = time_to_cooldown_1_degree / time_to_heatup_1_degree;
    double cool_down_time = getTimeToGoFromTempToTemp(extruder, outer_temp, temp1, during_printing);
    double time_needed_to_reach_temp1 = cool_down_time * (1.0 + time_ratio_heatup_cooldown);
    if (time_needed_to_reach_temp1 < limited_time_window)
    {
        result.cooling_time = cool_down_time;
        result.highest_temperature = temp1;
    }
    else 
    {
        result.cooling_time = limited_time_window * time_to_heatup_1_degree / (time_to_cooldown_1_degree + time_to_heatup_1_degree);
        result.highest_temperature = std::min(temp1, temp2 + result.cooling_time / time_to_cooldown_1_degree);
    }

    result.cooling_time += extra_cooldown_time;
    return result;
}


double Preheat::timeBeforeEndToInsertPreheatCommand_warmUp(double from_temp, unsigned int extruder, double temp, bool printing)
{
    if (temp > from_temp)
    {
        if (printing)
        {
            return (temp - from_temp) * (config_per_extruder[extruder].time_to_heatup_1_degree + config_per_extruder[extruder].heatup_cooldown_time_mod_while_printing); 
        }
        else 
        {
            return (temp - from_temp) * config_per_extruder[extruder].time_to_heatup_1_degree; 
        }
    }
    else 
    {
        if (printing)
        {
            return (from_temp - temp) * config_per_extruder[extruder].time_to_cooldown_1_degree; 
        }
        else 
        {
            return (from_temp - temp) * std::max(0.0, config_per_extruder[extruder].time_to_cooldown_1_degree - config_per_extruder[extruder].heatup_cooldown_time_mod_while_printing); 
        }
    }
}

}//namespace cura
