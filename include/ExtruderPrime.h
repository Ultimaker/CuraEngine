// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine
{

enum class ExtruderPrime
{
    None, // Do not prime at all for this extruder on this layer
    Support, // Just extrude a sparse pattern which purpose is to support the upper parts of the prime tower
    Prime, // Do an actual prime
};

} // namespace curaengine
