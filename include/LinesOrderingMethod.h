// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once


namespace cura
{

enum class LinesOrderingMethod
{
    Basic, // Lines are ordered by shortest distance
    Monotonic, // Lines are ordered so that they will always form a continuous print along a direction
    Interlaced, // Similar to monotonic, but with 2 passes so that adjacent lines will not be printed just after each other
};

} // namespace cura
