// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine
{
/*!
 * How to select the starting position of the infill
 */
enum class InfillStartEndPreference
{
    START_CLOSEST, // Just start at the vertex that is the closest to current position
    START_RANDOM, // Pick a random line to start at
    END_CLOSE_TO_SEAM // End close to the next wall seam, and split an infill line to provide a proper position if required
};

} // namespace curaengine
