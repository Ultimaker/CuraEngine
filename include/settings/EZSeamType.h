// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
enum class EZSeamType
{
    RANDOM,
    SHORTEST,
    USER_SPECIFIED,
    SHARPEST_CORNER,

    /* The 'Skirt/brim' type behaves like shortest, except it doesn't try to do tie-breaking for similar locations to
     * the last attempt, as that gives a different result when the seams are next to each other instead of on top.
     */
    SKIRT_BRIM,
    PLUGIN,
};

} // namespace cura
