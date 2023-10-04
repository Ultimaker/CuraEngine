// Copyright (c) 2023 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef EXTRUDERPRIME_H
#define EXTRUDERPRIME_H

namespace cura
{

enum class ExtruderPrime
{
    None, // Do not prime at all for this extruder on this layer
    Sparse, // Just extrude a sparse priming which purpose is to make the tower stronger
    Prime, // Do an actual prime
};

} // namespace cura
#endif // EXTRUDERPRIME_H
