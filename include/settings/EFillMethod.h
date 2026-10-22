// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming throughout the engine.
 * The different methods are used for top/bottom, support and sparse infill.
 */
enum class EFillMethod
{
    LINES,
    GRID,
    CUBIC,
    CUBICSUBDIV,
    TETRAHEDRAL,
    QUARTER_CUBIC,
    TRIANGLES,
    TRIHEXAGON,
    CONCENTRIC,
    ZIG_ZAG,
    CROSS,
    CROSS_3D,
    GYROID,
    LIGHTNING,
    HONEYCOMB,
    OCTAGON,
    NONE, // NOTE: Should remain second last! Before PLUGIN (Might be used in testing to enumerate the enum.)
    PLUGIN, // Place plugin after none to prevent it from being tested in the gtest suite.
};

} // namespace cura
