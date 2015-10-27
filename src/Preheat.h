#ifndef PREHEAT_H
#define PREHEAT_H

#include <cassert>
#include <algorithm> // max

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
        double time_to_heatup_1_degree; //!< average time it takes to heat up one degree (in the range of normal print temperatures and standby temperature)
        double time_to_cooldown_1_degree; //!< average time it takes to cool down one degree (in the range of normal print temperatures and standby temperature)
        
        double heatup_cooldown_time_mod_while_printing; //!< The time to be added to Preheat::time_to_heatup_1_degree and subtracted from Preheat::time_to_cooldown_1_degree to get the timings while printing

        double standby_temp = 150; // TODO

        double material_print_temperature; // default print temp
    
        FlowTempGraph flow_temp_graph;
    };

    std::vector<Config> config_per_extruder;
public:
    double getStandbyTemp(int extruder)
    {
        return config_per_extruder[extruder].standby_temp;
    }
    
    void setConfig(MeshGroup& settings)
    {
        for (int extruder_nr = 0; extruder_nr < settings.getExtruderCount(); extruder_nr++)
        {
            assert(settings.getExtruderTrain(extruder_nr) != nullptr);
            ExtruderTrain& extruder_train = *settings.getExtruderTrain(extruder_nr);
            config_per_extruder.emplace_back();
            Config& config = config_per_extruder.back();
            config.time_to_cooldown_1_degree = 0.5 ; // extruder_train.getSettingInSeconds("material_")
            config.time_to_heatup_1_degree = 0.5; // TODO
            config.heatup_cooldown_time_mod_while_printing = 0.1;
            config.standby_temp = 150; // TODO
            
            config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature");
            
            config.flow_temp_graph = extruder_train.getSettingAsFlowTempGraph("flow_temp_graph");
            config.flow_temp_graph.standby_temp = config.standby_temp;
        }
    }
private:
    /*!
     * Average ratio of cooling down one degree over heating up one degree
     * within the window between the standby and printing temperature during normal printing.
     */
    double timeToHeatFromStandbyToPrintTemp(unsigned int extruder, double temp)
    {
        return (temp - config_per_extruder[extruder].standby_temp) * config_per_extruder[extruder].time_to_heatup_1_degree; 
    }

public:
    
    double getTemp(unsigned int extruder, double flow)
    {
        return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature);
    }
    /*!
     * 
     * Assumes from_temp is approximately the same as @p temp
     * 
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     * \param temp The temperature to which to heat
     */
    double timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(double time_window, unsigned int extruder, double temp)
    {
        double time_ratio_cooldown_heatup = config_per_extruder[extruder].time_to_cooldown_1_degree / config_per_extruder[extruder].time_to_heatup_1_degree;
        double time_to_heat_from_standby_to_print_temp = timeToHeatFromStandbyToPrintTemp(extruder, temp);
        double time_needed_to_reach_standby_temp = time_to_heat_from_standby_to_print_temp * (1.0 + time_ratio_cooldown_heatup);
        if (time_needed_to_reach_standby_temp < time_window)
        {
            return time_to_heat_from_standby_to_print_temp;
        }
        else 
        {
            return time_window * config_per_extruder[extruder].time_to_heatup_1_degree / (config_per_extruder[extruder].time_to_cooldown_1_degree + config_per_extruder[extruder].time_to_heatup_1_degree);
        }
    }
    /*!
     * Calculate time needed to warm up the nozzle from a given temp to a given temp.
     * 
     * 
     * \param from_temp The temperature at which the nozzle was before
     * \param extruder The extruder used
     * \param temp The temperature to which to heat
     */
    double timeBeforeEndToInsertPreheatCommand_warmUp(double from_temp, unsigned int extruder, double temp, bool printing)
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
};

} // namespace cura 

#endif // PREHEAT_H