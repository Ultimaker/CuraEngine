// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine
{
enum class SectionType : int
{
    NA = -1,
    WALL = 1,
    INFILL = 2,
    SKIN = 3,
    SUPPORT = 4,
    ADHESION = 5,
    IRONING = 6,
    MESH = 7,
    DOTS = 8,
    CONCENTRIC_INFILL = 9
};
} // namespace curaengine
