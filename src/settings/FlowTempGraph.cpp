// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/FlowTempGraph.h"

#include <spdlog/spdlog.h>

namespace cura
{

double FlowTempGraph::getTemp(const double flow, const Temperature material_print_temperature, const bool flow_dependent_temperature) const
{
    if (! flow_dependent_temperature || data_.size() == 0)
    {
        return material_print_temperature;
    }
    if (data_.size() == 1)
    {
        return data_.front().temp_;
    }
    if (flow < data_.front().flow_)
    {
        spdlog::warn("Warning! Flow too low!");
        return data_.front().temp_;
    }
    const Datum* last_datum = &data_.front();
    for (unsigned int datum_idx = 1; datum_idx < data_.size(); datum_idx++)
    {
        const Datum& datum = data_[datum_idx];
        if (datum.flow_ >= flow)
        {
            return last_datum->temp_ + Temperature((datum.temp_ - last_datum->temp_) * (flow - last_datum->flow_) / (datum.flow_ - last_datum->flow_));
        }
        last_datum = &datum;
    }

    spdlog::warn("Warning! Flow too high!");
    return data_.back().temp_;
}

} // namespace cura
