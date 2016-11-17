#include "Preheat.h"

namespace cura 
{

void Preheat::setConfig(const MeshGroup& meshgroup)
{
    for (int extruder_nr = 0; extruder_nr < meshgroup.getExtruderCount(); extruder_nr++)
    {
        assert(meshgroup.getExtruderTrain(extruder_nr) != nullptr);
        const ExtruderTrain& extruder_train = *meshgroup.getExtruderTrain(extruder_nr);
        config_per_extruder.emplace_back();
        Config& config = config_per_extruder.back();
        double machine_nozzle_cool_down_speed = extruder_train.getSettingInSeconds("machine_nozzle_cool_down_speed");
        double machine_nozzle_heat_up_speed = extruder_train.getSettingInSeconds("machine_nozzle_heat_up_speed");
        double material_extrusion_cool_down_speed = extruder_train.getSettingInSeconds("material_extrusion_cool_down_speed");
        assert(material_extrusion_cool_down_speed < machine_nozzle_heat_up_speed && "The extrusion cooldown speed must be smaller than the heat up speed; otherwise the printing temperature cannot be reached!");
        config.time_to_cooldown_1_degree[0] = 1.0 / machine_nozzle_cool_down_speed;
        config.time_to_heatup_1_degree[0] = 1.0 / machine_nozzle_heat_up_speed;
        config.time_to_cooldown_1_degree[1] = 1.0 / (machine_nozzle_cool_down_speed + material_extrusion_cool_down_speed);
        config.time_to_heatup_1_degree[1] = 1.0 / (machine_nozzle_heat_up_speed - material_extrusion_cool_down_speed);
        config.standby_temp = extruder_train.getSettingInSeconds("material_standby_temperature");

        config.min_time_window = extruder_train.getSettingInSeconds("machine_min_cool_heat_time_window");

        config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature");
        config.material_print_temperature_layer_0 = extruder_train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        config.material_initial_print_temperature = extruder_train.getSettingInDegreeCelsius("material_initial_print_temperature");
        config.material_final_print_temperature = extruder_train.getSettingInDegreeCelsius("material_final_print_temperature");

        config.flow_dependent_temperature = extruder_train.getSettingBoolean("material_flow_dependent_temperature"); 

        config.flow_temp_graph = extruder_train.getSettingAsFlowTempGraph("material_flow_temp_graph"); // [[0.1,180],[20,230]]
    }
}

double Preheat::getTimeToGoFromTempToTemp(int extruder, double temp_before, double temp_after, bool during_printing)
{
    Config& config = config_per_extruder[extruder];
    double time;
    if (temp_after > temp_before)
    {
        time = (temp_after - temp_before) * config.time_to_heatup_1_degree[during_printing];
    }
    else
    {
        time = (temp_before - temp_after) * config.time_to_cooldown_1_degree[during_printing];
    }
    return std::max(0.0, time);
}

double Preheat::getTemp(unsigned int extruder, double flow, bool is_initial_layer)
{
    if (is_initial_layer)
    {
        return config_per_extruder[extruder].material_print_temperature_layer_0;
    }
    return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature, config_per_extruder[extruder].flow_dependent_temperature);
}

Preheat::WarmUpResult Preheat::getWarmUpPointAfterCoolDown(double time_window, unsigned int extruder, double temp_start, double temp_mid, double temp_end, bool during_printing)
{
    WarmUpResult result;
    const Config& config = config_per_extruder[extruder];
    double time_to_cooldown_1_degree = config.time_to_cooldown_1_degree[during_printing];
    double time_to_heatup_1_degree = config.time_to_heatup_1_degree[during_printing];
    result.total_time_window = time_window;

    //                    ,temp_end
    //                   /                                    .
    //     ,temp_start  /                                     .
    //      \ ' ' ' ' '/ ' ' '> outer_temp                    .
    //       \________/                                       .
    //               "-> temp_mid
    //      ^^^^^^^^^^
    //      limited_time_window
    double outer_temp;
    double limited_time_window;
    if (temp_start < temp_end)
    { // extra time needed during heating
        double extra_heatup_time = (temp_end - temp_start) * time_to_heatup_1_degree;
        result.heating_time = extra_heatup_time;
        limited_time_window = time_window - extra_heatup_time;
        outer_temp = temp_start;
    }
    else
    {
        double extra_cooldown_time = (temp_start - temp_end) * time_to_cooldown_1_degree;
        result.heating_time = 0;
        limited_time_window = time_window - extra_cooldown_time;
        outer_temp = temp_end;
    }
    double time_ratio_cooldown_heatup = time_to_cooldown_1_degree / time_to_heatup_1_degree;
    double time_to_heat_from_standby_to_print_temp = getTimeToGoFromTempToTemp(extruder, temp_mid, outer_temp, during_printing);
    double time_needed_to_reach_standby_temp = time_to_heat_from_standby_to_print_temp * (1.0 + time_ratio_cooldown_heatup);
    if (time_needed_to_reach_standby_temp < limited_time_window)
    {
        result.heating_time += time_to_heat_from_standby_to_print_temp;
        result.lowest_temperature = temp_mid;
    }
    else 
    {
        result.heating_time += limited_time_window * time_to_heatup_1_degree / (time_to_cooldown_1_degree + time_to_heatup_1_degree);
        result.lowest_temperature = std::max(temp_mid, temp_end - result.heating_time / time_to_heatup_1_degree);
    }
    return result;
}

Preheat::CoolDownResult Preheat::getCoolDownPointAfterWarmUp(double time_window, unsigned int extruder, double temp_start, double temp_mid, double temp_end, bool during_printing)
{
    CoolDownResult result;
    const Config& config = config_per_extruder[extruder];
    double time_to_cooldown_1_degree = config.time_to_cooldown_1_degree[during_printing];
    double time_to_heatup_1_degree = config.time_to_heatup_1_degree[during_printing];

    result.total_time_window = time_window;

    //      limited_time_window
    //      ^^^^^^^^^^
    //                 _> temp_mid
    //       /""""""""\                                       .
    //      / . . . . .\ . . .> outer_temp                    .
    //     ^temp_start  \                                     .
    //                   \                                    .
    //                    ^temp_end
    double outer_temp;
    double limited_time_window;
    if (temp_start < temp_end)
    { // extra time needed during heating
        double extra_heatup_time = (temp_end - temp_start) * time_to_heatup_1_degree;
        result.cooling_time = 0;
        limited_time_window = time_window - extra_heatup_time;
        outer_temp = temp_start;
    }
    else
    {
        double extra_cooldown_time = (temp_start - temp_end) * time_to_cooldown_1_degree;
        result.cooling_time = extra_cooldown_time;
        limited_time_window = time_window - extra_cooldown_time;
        outer_temp = temp_end;
    }
    double time_ratio_cooldown_heatup = time_to_cooldown_1_degree / time_to_heatup_1_degree;
    double cool_down_time = getTimeToGoFromTempToTemp(extruder, temp_mid, outer_temp, during_printing);
    double time_needed_to_reach_temp1 = cool_down_time * (1.0 + time_ratio_cooldown_heatup);
    if (time_needed_to_reach_temp1 < limited_time_window)
    {
        result.cooling_time += cool_down_time;
        result.highest_temperature = temp_mid;
    }
    else 
    {
        result.cooling_time += limited_time_window * time_to_heatup_1_degree / (time_to_cooldown_1_degree + time_to_heatup_1_degree);
        result.highest_temperature = std::min(temp_mid, temp_end + result.cooling_time / time_to_cooldown_1_degree);
    }

    return result;
}


}//namespace cura
