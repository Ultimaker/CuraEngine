// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "RetractionConfig.h"
#include "WipeScriptConfig.h"
#include "types/Coord_t.h"

namespace curaengine
{
struct RetractionAndWipeConfig
{
    WipeScriptConfig wipe_config;

    RetractionConfig retraction_config;
    types::coord_t retraction_hop_after_extruder_switch;
    double switch_extruder_extra_prime_amount;
    RetractionConfig extruder_switch_retraction_config;
};

} // namespace curaengine
