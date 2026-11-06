// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming throughout the engine.
 * The different methods are used for top/bottom, support and sparse infill.
 */
enum class EFillMethod
{
    Lines,
    Grid,
    Cubic,
    CubicSubdiv,
    Tetrahedral,
    QuarterCubic,
    Triangles,
    TriHexagon,
    Concentric,
    ZigZag,
    Cross,
    Cross3D,
    Gyroid,
    Lightning,
    Honeycomb,
    Octagon,
    None, // NOTE: Should remain second last! Before Plugin (Might be used in testing to enumerate the enum.)
    Plugin, // Place plugin after none to prevent it from being tested in the gtest suite.
};

} // namespace curaengine::types
