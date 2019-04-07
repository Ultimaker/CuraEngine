#ifndef FLOW_TEMP_GRAPH
#define FLOW_TEMP_GRAPH

#include <cassert>
#include <vector>

#include "types/Temperature.h"

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
        const double flow; //!< The flow in mm^3/s
        const Temperature temp; //!< The temperature in *C
        Datum(const double flow, const Temperature temp)
        : flow(flow)
        , temp(temp)
        {}
    };

    std::vector<Datum> data; //!< The points of the graph between which the graph is linearly interpolated

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
    double getTemp(const double flow, const Temperature material_print_temperature, const bool flow_dependent_temperature) const;
};

} // namespace cura

#endif // FLOW_TEMP_GRAPH