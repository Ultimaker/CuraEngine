//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "RetractionConfig.h"
#include "WipeScriptConfig.h"
#include "utils/Coord_t.h"

namespace cura
{
struct RetractionAndWipeConfig
{
    WipeScriptConfig wipe_config;

    RetractionConfig retraction_config;
    coord_t retraction_hop_after_extruder_switch;
    double switch_extruder_extra_prime_amount;
    RetractionConfig extruder_switch_retraction_config;
};

} // namespace cura
