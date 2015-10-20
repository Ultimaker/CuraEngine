#ifndef PREHEAT_H
#define PREHEAT_H

#include <cassert>

#include "utils/logoutput.h"
#include "MeshGroup.h"

#include "FlowTempGraph.h"

namespace cura 
{



/*!
 * Class for computing heatup and cooldown times used for computing the time the printer needs to heat up to the printing temperature.
 */
class Preheat 
{
    class Config
    {
    public:
        double time_to_heatup_1_degree;
        double time_to_cooldown_1_degree;

        double idle_temp = 150; // TODO

        double material_print_temperature; // default print temp
    
        FlowTempGraph flow_temp_graph;
    };

    std::vector<Config> config_per_extruder;
public:
    void setConfig(MeshGroup& settings)
    {
        for (int extruder_nr = 0; extruder_nr < settings.getExtruderCount(); extruder_nr++)
        {
            assert(settings.getExtruderTrain(extruder_nr) != nullptr);
            ExtruderTrain& extruder_train = *settings.getExtruderTrain(extruder_nr);
            config_per_extruder.emplace_back();
            Config& config = config_per_extruder.back();
            config.time_to_cooldown_1_degree = 0.6; // TODO
            config.time_to_cooldown_1_degree = 0.4; // TODO
            config.idle_temp = 150; // TODO
            
            config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature");
            
            config.flow_temp_graph = extruder_train.getSettingAsFlowTempGraph("flow_temp_graph");
        }
    }
private:
    double timeToHeatFromIdleToPrintTemp(unsigned int extruder, double temp)
    {
        return (temp - config_per_extruder[extruder].idle_temp) * config_per_extruder[extruder].time_to_heatup_1_degree; 
    }

    /*!
     * Average ratio of cooling down one degree over heating up one degree
     * within the window between the idle and printing temperature during normal printing.
     */
    double timeRatioCooldownHeatup(unsigned int extruder)
    {
        return config_per_extruder[extruder].time_to_cooldown_1_degree / config_per_extruder[extruder].time_to_heatup_1_degree; 
    }

public:
    
    double getTemp(unsigned int extruder, double flow)
    {
        return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature);
    }
    /*!
     * 
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     */
    double timeBeforeEndToInsertPreheatCommand(double time_window, unsigned int extruder, double temp)
    {
        double time_ratio_cooldown_heatup = timeRatioCooldownHeatup(extruder);
        double time_to_heat_from_idle_to_print_temp = timeToHeatFromIdleToPrintTemp(extruder, temp);
        double time_needed_to_reach_idle_temp = time_to_heat_from_idle_to_print_temp * (1.0 + time_ratio_cooldown_heatup);
        if (time_needed_to_reach_idle_temp < time_window)
        {
            return time_to_heat_from_idle_to_print_temp;
        }
        else 
        {
            return time_window * config_per_extruder[extruder].time_to_heatup_1_degree / (config_per_extruder[extruder].time_to_cooldown_1_degree + config_per_extruder[extruder].time_to_heatup_1_degree);
        }
    }
};

} // namespace cura 

#endif // PREHEAT_H