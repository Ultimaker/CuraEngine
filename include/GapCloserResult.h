// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>

#include "utils/Coord_t.h"

namespace curaengine
{
class GapCloserResult
{
public:
    ::curaengine::utils::coord_t len = 0;
    size_t polygonIdx = 0;
    size_t pointIdxA = 0;
    size_t pointIdxB = 0;
    bool AtoB = false;
};

} // namespace curaengine
