// Copyright (c) 2023 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXTRUDERUSE_H
#define EXTRUDERUSE_H

#include <stddef.h>

#include "ExtruderPrime.h"

namespace cura
{

struct ExtruderUse
{
    size_t extruder_nr;
    ExtruderPrime prime;
};

} // namespace cura
#endif // EXTRUDERUSE_H
