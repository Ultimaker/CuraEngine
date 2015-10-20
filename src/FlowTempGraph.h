#ifndef FLOW_TEMP_GRAPH
#define FLOW_TEMP_GRAPH

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
        Datum(double flow, double temp)
        : flow(flow)
        , temp(temp)
        {}
    };
    std::vector<Datum> data;

    
    double getTemp(double flow, double material_print_temperature)
    {
        if (data.size() == 0)
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