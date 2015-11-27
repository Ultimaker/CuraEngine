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
 * Class for computing heatup and cooldown times used for computing the time the printer needs to heat up to a printing temperature.
 */
class Preheat 
{
    /*!
     * The nozzle and material temperature settings for an extruder train.
     */
    class Config
    {
    public:
        double time_to_heatup_1_degree; //!< average time it takes to heat up one degree (in the range of normal print temperatures and standby temperature)
        double time_to_cooldown_1_degree; //!< average time it takes to cool down one degree (in the range of normal print temperatures and standby temperature)
        
        double heatup_cooldown_time_mod_while_printing; //!< The time to be added to Preheat::time_to_heatup_1_degree and subtracted from Preheat::time_to_cooldown_1_degree to get the timings while printing

        double standby_temp; //!< The temperature at which the nozzle rests when it is not printing.

        double material_print_temperature; //!< default print temp (backward compatilibily)
        
        bool flow_dependent_temperature; //!< Whether to make the temperature dependent on flow
    
        FlowTempGraph flow_temp_graph; //!< The graph linking flows to corresponding temperatures
    };

    std::vector<Config> config_per_extruder;//!< the nozzle and material temperature settings for each extruder train.
public:
    /*!
     * Get the standby temperature of an extruder train
     * \param extruder the extruder train for which to get the standby tmep
     * \return the standby temp
     */
    double getStandbyTemp(int extruder)
    {
        return config_per_extruder[extruder].standby_temp;
    }
    
    /*!
     * Set the nozzle and material temperature settings for each extruder train.
     */
    void setConfig(MeshGroup& settings)
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
            
            config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature"); // 220
            
            config.flow_dependent_temperature = extruder_train.getSettingBoolean("material_flow_dependent_temperature"); 
            
            config.flow_temp_graph = extruder_train.getSettingAsFlowTempGraph("material_flow_temp_graph"); // [[0.1,180],[20,230]]
        }
    }
    
    bool usesFlowDependentTemp(int extruder_nr)
    {
        return config_per_extruder[extruder_nr].flow_dependent_temperature;
    }
private:
    /*!
     * Calculate time to heat up from standby temperature to a given temperature.
     * Assumes @p temp is higher than the standby temperature.
     * 
     * \param extruder The extruder for which to get the time
     * \param temp The temperature to be reached
     */
    double timeToHeatFromStandbyToPrintTemp(unsigned int extruder, double temp)
    {
        return (temp - config_per_extruder[extruder].standby_temp) * config_per_extruder[extruder].time_to_heatup_1_degree; 
    }

public:
    
    /*!
     * Get the optimal temperature corresponding to a given average flow.
     * \param extruder The extruder train
     * \param flow The flow for which to get the optimal temperature
     * \return The corresponding optimal temperature
     */
    double getTemp(unsigned int extruder, double flow)
    {
        return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature, config_per_extruder[extruder].flow_dependent_temperature);
    }
    
    /*!
     * Decide when to start warming up again after starting to cool down towards the standby temperature.
     * Two cases are considered: 
     * the case where the standby temperature is reached  \__/    .
     * and the case where it isn't  \/    .
     * 
     * IT is assumed that the printer is not printing during this cool down and warm up time.
     * 
     * Assumes from_temp is approximately the same as @p temp
     * 
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     * \param temp The temperature to which to heat
     * \return The time before the end of the @p time_window to insert the preheat command
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
     * If the printer is printing in the mean time the warming up will take longer.
     * 
     * 
     * \param from_temp The temperature at which the nozzle was before
     * \param extruder The extruder used
     * \param temp The temperature to which to heat
     * \param printing Whether the printer is printing in the time to heat up the nozzle
     * \return The time needed to reach the desired temperature (@p temp)
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