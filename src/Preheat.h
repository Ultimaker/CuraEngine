//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PREHEAT_H
#define PREHEAT_H

#include <cassert>
#include <algorithm> // max

#include "settings/types/Duration.h"
#include "settings/types/Temperature.h"

namespace cura 
{

class Ratio;

/*!
 * Class for computing heatup and cooldown times used for computing the time the printer needs to heat up to a printing temperature.
 */
class Preheat 
{
public:
    /*!
     * The type of result when computing when to start heating up a nozzle before it's going to be used again.
     */
    struct WarmUpResult
    {
        Duration total_time_window; //!< The total time in which cooling and heating takes place.
        Duration heating_time; //!< The total time needed to heat to the required temperature.
        Temperature lowest_temperature; //!< The lower temperature from which heating starts.
    };

    /*!
     * The type of result when computing when to start cooling down a nozzle before it's not going to be used again.
     */
    struct CoolDownResult
    {
        Duration total_time_window; //!< The total time in which heating and cooling takes place.
        Duration cooling_time; //!< The total time needed to cool down to the required temperature.
        Temperature highest_temperature; //!< The upper temperature from which cooling starts.
    };

    /*!
     * Get the optimal temperature corresponding to a given average flow,
     * or the initial layer temperature.
     * 
     * \param extruder The extruder train
     * \param flow The flow for which to get the optimal temperature
     * \param is_initial_layer Whether the initial layer temperature should be returned instead of flow-based temperature
     * \return The corresponding optimal temperature
     */
    Temperature getTemp(const size_t extruder, const Ratio& flow, const bool is_initial_layer);

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
    Duration getTimeToGoFromTempToTemp(const size_t extruder, const Temperature& temp_before, const Temperature& temp_after, const bool during_printing);
};

} // namespace cura 

#endif // PREHEAT_H