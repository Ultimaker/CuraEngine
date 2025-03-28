// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "settings/Settings.h"
#include "types/LayerIndex.h"
#include "utils/Coord_t.h"

namespace cura
{

class ExtruderSettings : public Settings
{
public:
    explicit ExtruderSettings() = default;

    coord_t getLineWidth(const LayerIndex& layer_nr, const std::string& line_width_setting_name = "line_width") const;
};

} // namespace cura
