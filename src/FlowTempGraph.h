#ifndef FLOW_TEMP_GRAPH
#define FLOW_TEMP_GRAPH

#include <cassert>

#include "utils/logoutput.h"

namespace cura
{

/*!
 * Class representing a graph matching a flow to a temperature.
 * The graph generally consists of several linear line segments between points at which the temperature and flow are matched.
 */
class FlowTempGraph
{
public:
    struct Datum
    {
        double flow; //!< The flow in mm^3/s
        double temp; //!< The temperature in *C
        Datum(double flow, double temp)
        : flow(flow)
        , temp(temp)
        {}
    };
    std::vector<Datum> data; //!< The points of the graph between which the graph is linearly interpolated

    FlowTempGraph()
    {}
    
    /*!
     * Get the temperature corresponding to a specific flow.
     * 
     * For flows outside of the chart, the temperature at the minimal or maximal flow is returned.
     * When the graph is empty, the @p material_print_temperature is returned.
     * 
     * \param flow the flow in mm^3/s
     * \param material_print_temperature The default printing temp (backward compatibility for when the graph fails)
     * \return the corresponding temp
     */
    double getTemp(double flow, double material_print_temperature, bool flow_dependent_temperature)
    {
        if (!flow_dependent_temperature || data.size() == 0)
        {
            return material_print_temperature;
        }
        if (data.size() == 1)
        {
            return data.front().temp;
        }
        if (flow < data.front().flow)
        {
            logError("Warning! Flow too low!\n"); // TODO
            return data.front().temp;
        }
        Datum* last_datum = &data.front();
        for (unsigned int datum_idx = 1; datum_idx < data.size(); datum_idx++)
        {
            Datum& datum = data[datum_idx];
            if (datum.flow >= flow)
            {
                return last_datum->temp + (datum.temp - last_datum->temp) * (flow - last_datum->flow) / (datum.flow - last_datum->flow);
            }
            last_datum = &datum;
        }
        
        logError("Warning! Flow too high!\n"); // TODO
        return data.back().temp;
    };
};



} // namespace cura

#endif // FLOW_TEMP_GRAPH