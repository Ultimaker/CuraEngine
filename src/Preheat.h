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

        double min_time_window; //!< Minimal time (in seconds) to allow an extruder to cool down and then warm up again.

        double material_print_temperature; //!< default print temp (backward compatilibily)

        double material_print_temperature_layer_0; //!< initial layer print temp
        
        bool flow_dependent_temperature; //!< Whether to make the temperature dependent on flow
    
        FlowTempGraph flow_temp_graph; //!< The graph linking flows to corresponding temperatures
    };

    std::vector<Config> config_per_extruder;//!< the nozzle and material temperature settings for each extruder train.
public:
    /*!
     * The type of result when computing when to start heating up a nozzle before it's going to be used again.
     */
    struct WarmUpResult
    {
        double total_time_window; //!< The total time in which cooling and heating takes place.
        double heating_time; //!< The total time needed to heat to the required temperature.
        double lowest_temperature; //!< The lower temperature from which heating starts.
    };

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
     * Get the time it takes to heat up one degree celsius
     * 
     * \param extruder the extruder train for which to get time it takes to heat up one degree celsius
     * \return the time it takes to heat up one degree celsius
     */
    double getTimeToHeatup1Degree(int extruder)
    {
        return config_per_extruder[extruder].time_to_heatup_1_degree;
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

            config.min_time_window = extruder_train.getSettingInSeconds("machine_min_cool_heat_time_window");

            config.material_print_temperature = extruder_train.getSettingInDegreeCelsius("material_print_temperature"); // 220

            config.material_print_temperature_layer_0 = extruder_train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
            
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
     * Get the optimal temperature corresponding to a given average flow,
     * or the initial layer temperature.
     * 
     * \param extruder The extruder train
     * \param flow The flow for which to get the optimal temperature
     * \param is_initial_layer Whether the initial layer temperature should be returned instead of flow-based temperature
     * \return The corresponding optimal temperature
     */
    double getTemp(unsigned int extruder, double flow, bool is_initial_layer)
    {
        if (is_initial_layer)
        {
            return config_per_extruder[extruder].material_print_temperature_layer_0;
        }
        return config_per_extruder[extruder].flow_temp_graph.getTemp(flow, config_per_extruder[extruder].material_print_temperature, config_per_extruder[extruder].flow_dependent_temperature);
    }

    /*!
     * Return the minimal time window of a specific extruder for letting an unused extruder cool down and warm up again
     * \param extruder The extruder for which to get the minimal time window
     * \return the minimal time window of a specific extruder for letting an unused extruder cool down and warm up again
     */
    double getMinimalTimeWindow(unsigned int extruder)
    {
        return config_per_extruder[extruder].min_time_window;
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
     * \return The time before the end of the @p time_window to insert the preheat command and the temperature from which the heating starts
     */
    WarmUpResult timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(double time_window, unsigned int extruder, double temp)
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