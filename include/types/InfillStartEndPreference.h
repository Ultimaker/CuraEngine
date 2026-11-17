// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * How to select the starting position of the infill
 */
enum class InfillStartEndPreference
{
    StartClosest, // Just start at the vertex that is the closest to current position
    StartRandom, // Pick a random line to start at
    EndCloseToSeam // End close to the next wall seam, and split an infill line to provide a proper position if required
};

} // namespace curaengine::types
