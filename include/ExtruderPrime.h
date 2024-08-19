// Copyright (c) 2023 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXTRUDERPRIME_H
#define EXTRUDERPRIME_H

namespace cura
{

enum class ExtruderPrime
{
    None, // Do not prime at all for this extruder on this layer
    Support, // Just extrude a sparse pattern which purpose is to support the upper parts of the prime tower
    Prime, // Do an actual prime
};

} // namespace cura
#endif // EXTRUDERPRIME_H
