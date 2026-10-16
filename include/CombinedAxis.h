// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

#include "utils/Flags.h"

namespace cura::CombinedAxis
{

enum Enum : uint8_t
{
    XY = 0b001, //<! The horizontal plan, composed of the X and Y axes
    Z = 0b010, //<! The vertical up-and-down Z axis
    E = 0b100 //<! The extrusion "virtual" E axis
};

static constexpr size_t Count{ 3 };

using Flags = Flags<Enum>;

static constexpr Flags All{ Flags(XY) | Flags(Z) | Flags(E) };

} // namespace cura::CombinedAxis
