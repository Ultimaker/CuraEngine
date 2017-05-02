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
        double time_to_heatup_1_degree[2]; //!< average time it takes to heat up one degree (in the range of normal print temperatures and standby temperature), while not-printing and while printing
        double time_to_cooldown_1_degree[2]; //!< average time it takes to cool down one degree (in the range of normal print temperatures and standby temperature), while not-printing and while printing

        double standby_temp; //!< The temperature at which the nozzle rests when it is not printing.

        double min_time_window; //!< Minimal time (in seconds) to allow an extruder to cool down and then warm up again.

        double material_print_temperature; //!< default print temp (backward compatilibily)

        double material_print_temperature_layer_0; //!< initial layer print temp

        double material_initial_print_temperature; //!< print temp when first starting to extrude after a layer switch

        double material_final_print_temperature; //!< print temp at the end of all extrusion moves of an extruder to which it's cooled down just before - during the extrusion

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
     * The type of result when computing when to start cooling down a nozzle before it's not going to be used again.
     */
    struct CoolDownResult
    {
        double total_time_window; //!< The total time in which heating and cooling takes place.
        double cooling_time; //!< The total time needed to cool down to the required temperature.
        double highest_temperature; //!< The upper temperature from which cooling starts.
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
     * \param during_printing whether the heating takes time during printing or when idle
     * \return the time it takes to heat up one degree celsius
     */
    double getTimeToHeatup1Degree(int extruder, bool during_printing)
    {
        return config_per_extruder[extruder].time_to_heatup_1_degree[during_printing];
    }

    /*!
     * Get the initial print temperature when starting to extrude.
     * \param during_printing whether the heating takes time during printing or when idle
     */
    double getInitialPrintTemp(int extruder)
    {
        return config_per_extruder[extruder].material_initial_print_temperature;
    }

    /*!
     * Get the final print temperature at the end of all extrusion moves with the current extruder
     */
    double getFinalPrintTemp(int extruder)
    {
        return config_per_extruder[extruder].material_final_print_temperature;
    }

    /*!
     * Set the nozzle and material temperature settings for each extruder train.
     * \param meshgroup Where to get settings from
     */
    void setConfig(const MeshGroup& meshgroup);

    bool usesFlowDependentTemp(int extruder_nr)
    {
        return config_per_extruder[extruder_nr].flow_dependent_temperature;
    }
    /*!
     * Get the optimal temperature corresponding to a given average flow,
     * or the initial layer temperature.
     * 
     * \param extruder The extruder train
     * \param flow The flow for which to get the optimal temperature
     * \param is_initial_layer Whether the initial layer temperature should be returned instead of flow-based temperature
     * \return The corresponding optimal temperature
     */
    double getTemp(unsigned int extruder, double flow, bool is_initial_layer);

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
     * Decide when to start warming up again after starting to cool down towards \p temp_mid.
     * Two cases are considered: 
     * the case where the standby temperature is reached  \__/    .
     * and the case where it isn't  \/    .
     * 
     * \warning it is assumed that \p temp_mid is lower than both \p temp_start and \p temp_end. If not somewhat weird results may follow.
     * 
    //                    ,temp_end
    //                   /                                    .
    //     ,temp_start  /                                     .
    //      \          /                                      .
    //       \________/                                       .
    //               "-> temp_mid
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     * \param temp_start The temperature from which to start cooling down
     * \param temp_mid The temeprature to which we try to cool down
     * \param temp_end The temperature to which we need to have heated up at the end of the \p time_window
     * \param during_printing Whether the warming up and cooling down is performed during printing
     * \return The time before the end of the @p time_window to insert the preheat command and the temperature from which the heating starts
     */
    WarmUpResult getWarmUpPointAfterCoolDown(double time_window, unsigned int extruder, double temp_start, double temp_mid, double temp_end, bool during_printing);

    /*!
     * Decide when to start cooling down again after starting to warm up towards the \p temp_mid
     * Two cases are considered: 
     * the case where the temperature is reached  /"""\    .
     * and the case where it isn't  /\    .
     * 
     * \warning it is assumed that \p temp_mid is higher than both \p temp_start and \p temp_end. If not somewhat weird results may follow.
     * 
    //               _> temp_mid
    //       /""""""""\                                       .
    //      /          \                                      .
    //     ^temp_start  \                                     .
    //                   \                                    .
    //                    ^temp_end
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     * \param temp_start The temperature from which to start heating up
     * \param temp_mid The temeprature to which we try to heat up
     * \param temp_end The temperature to which we need to have cooled down after \p time_window time
     * \param during_printing Whether the warming up and cooling down is performed during printing
     * \return The time before the end of the \p time_window to insert the preheat command and the temperature from which the cooling starts
     */
    CoolDownResult getCoolDownPointAfterWarmUp(double time_window, unsigned int extruder, double temp_start, double temp_mid, double temp_end, bool during_printing);

    /*!
     * Get the time to go from one temperature to another temperature
     * \param extruder The extruder number for which to perform the heatup / cooldown
     * \param temp_before The before temperature
     * \param temp_after The after temperature
     * \param during_printing Whether the planned cooldown / warmup occurs during printing or while in standby mode
     * \return The time needed
     */
    double getTimeToGoFromTempToTemp(int extruder, double temp_before, double temp_after, bool during_printing);
};

} // namespace cura 

#endif // PREHEAT_H