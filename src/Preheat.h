#ifndef PREHEAT_H
#define PREHEAT_H

#include <cassert>

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
        assert(data.size() > 0);
        if (data.size() == 1)
        {
            return data.front().temp;
        }
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
        Datum* last_datum = &data.front();
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
    class Config
    {
    public:
        double time_to_heatup_1_degree = 0.6; // TODO 
        double time_to_cooldown_1_degree = 0.4; // TODO

        double idle_temp = 150; // TODO

        FlowTempGraph flow_temp_graph;
    };

    std::vector<Config> config_per_extruder;

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
        return config_per_extruder[extruder].flow_temp_graph.getTemp(flow);
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