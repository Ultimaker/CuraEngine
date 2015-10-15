#ifndef PREHEAT_H
#define PREHEAT_H

#include "utils/logoutput.h"

namespace cura 
{

class FlowTempGraph
{
public:
    struct Datum
    {
        double flow;
        double temp;
    };
    std::vector<Datum> data;
    
    double getTemp(double flow)
    {
        if (flow < data.front().flow)
        {
            logError("flow too low"); // TODO
            return data.front().temp;
        }
        if (flow > data.back().flow)
        {
            logError("flow too high"); // TODO
            return data.back().temp;
        }
        Datum* last_datum = data.front();
        for (unsigned int datum_idx = 1; datum_idx < data.size(); datum_idx++)
        {
            Datum& datum = data[datum_idx];
            if (datum.flow > flow)
            {
                return last_datum->temp + (datum.temp - last_datum->temp) * (flow - last_datum->flow) / (datum.flow - last_datum->flow);
            }
            last_datum = &datum;
        }
        
    };
};

/*!
 * Class for computing heatup and cooldown times used for computing the time the printer needs to heat up to the printing temperature.
 */
class Preheat 
{
    double time_to_heatup_1_degree = 0.6; // TODO per extruder
    double time_to_cooldown_1_degree = 0.4; // TODO
    
    double idle_temp = 150; // TODO
    double printing_temp = 230; // TODO
    
    double timeToHeatFromIdleToPrintTemp(unsigned int extruder)
    {
        return (printing_temp - idle_temp) * time_to_heatup_1_degree; 
    }

    /*!
     * Average ratio of cooling down one degree over heating up one degree
     * within the window between the idle and printing temperature during normal printing.
     */
    double timeRatioCooldownHeatup(unsigned int extruder)
    {
        return time_to_cooldown_1_degree / time_to_heatup_1_degree; 
    }

public:
    /*!
     * 
     * \param window_time The time window within which the cooldown and heat up must take place.
     * \param extruder The extruder used
     */
    double timeBeforeEndToInsertPreheatCommand(double time_window, unsigned int extruder)
    {
        double time_ratio_cooldown_heatup = timeRatioCooldownHeatup(extruder);
        double time_needed_to_reach_idle_temp = timeToHeatFromIdleToPrintTemp(extruder) * (1.0 + time_ratio_cooldown_heatup);
        if (time_needed_to_reach_idle_temp < time_window)
        {
            return timeToHeatFromIdleToPrintTemp(extruder);
        }
        else 
        {
            return time_window * time_to_heatup_1_degree / (time_to_cooldown_1_degree + time_to_heatup_1_degree);
        }
    }
};

} // namespace cura 

#endif // PREHEAT_H