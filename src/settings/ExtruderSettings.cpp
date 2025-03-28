// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/ExtruderSettings.h"

#include <settings/types/Ratio.h>

namespace cura
{

coord_t ExtruderSettings::getLineWidth(const LayerIndex& layer_nr, const std::string& line_width_setting_name) const
{
    coord_t line_width = get<coord_t>(line_width_setting_name);

    if (layer_nr == 0)
    {
        line_width *= get<Ratio>("initial_layer_line_width_factor");
    }

    return line_width;
}

} // namespace cura
