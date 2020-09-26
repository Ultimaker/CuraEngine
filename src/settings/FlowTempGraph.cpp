//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FlowTempGraph.h"
#include "../utils/logoutput.h"

namespace cura
{

double FlowTempGraph::getTemp(const double flow, const Temperature material_print_temperature, const bool flow_dependent_temperature) const
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
        logWarning("Warning! Flow too low!\n"); // TODO
        return data.front().temp;
    }
    const Datum* last_datum = &data.front();
    for (unsigned int datum_idx = 1; datum_idx < data.size(); datum_idx++)
    {
        const Datum& datum = data[datum_idx];
        if (datum.flow >= flow)
        {
            return last_datum->temp + Temperature((datum.temp - last_datum->temp) * (flow - last_datum->flow) / (datum.flow - last_datum->flow));
        }
        last_datum = &datum;
    }

    logWarning("Warning! Flow too high!\n"); // TODO
    return data.back().temp;
}

} //namespace cura
