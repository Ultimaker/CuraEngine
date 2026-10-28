// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "settings/types/Ratio.h"

namespace curaengine
{
struct ZHopAntiOozing
{
    double amount{ 0 }; //!< The absolute amount of retracted material (in mm) to be reached while the nozzle is moved up/down
    Ratio ratio; //!< The ratio of the full z-hop move that contains a retract/prime
};

} // namespace curaengine
