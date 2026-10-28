// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>

#include "ExtruderPrime.h"

namespace curaengine
{

struct ExtruderUse
{
    size_t extruder_nr;
    ExtruderPrime prime;
};

} // namespace curaengine
