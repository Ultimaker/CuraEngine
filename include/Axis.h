// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

#include "utils/Flags.h"

namespace cura::Axis
{

enum Enum : uint8_t
{
    X = 0b0001, //<! The horizontal left-and-right X axis
    Y = 0b0010, //<! The horizontal front-and-rear X axis
    Z = 0b0100, //<! The vertical up-and-down Z axis
    E = 0b1000 //<! The extrusion "virtual" E axis
};

static constexpr size_t Count{ 4 };

using Flags = Flags<Enum>;

static constexpr Flags All{ Flags(X) | Flags(Y) | Flags(Z) | Flags(E) };

} // namespace cura::Axis
