// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>

#include "utils/Coord_t.h"

namespace cura
{
class GapCloserResult
{
public:
    coord_t len = 0;
    size_t polygonIdx = 0;
    size_t pointIdxA = 0;
    size_t pointIdxB = 0;
    bool AtoB = false;
};

} // namespace cura
