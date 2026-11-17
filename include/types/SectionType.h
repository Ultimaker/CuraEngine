// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
enum class SectionType : int
{
    Na = -1,
    Wall = 1,
    Infill = 2,
    Skin = 3,
    Support = 4,
    Adhesion = 5,
    Ironing = 6,
    Mesh = 7,
    Dots = 8,
    ConcentricInfill = 9
};
} // namespace curaengine::types
