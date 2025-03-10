// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{

enum class PrintFeatureType : unsigned char
{
    NoneType = 0, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall = 1,
    InnerWall = 2,
    Skin = 3,
    Support = 4,
    SkirtBrim = 5,
    Infill = 6,
    SupportInfill = 7,
    MoveCombing = 8,
    MoveRetraction = 9,
    SupportInterface = 10,
    PrimeTower = 11,
    Roof = 12,
    NumPrintFeatureTypes = 13 // this number MUST be the last one because other modules will
                              // use this symbol to get the total number of types, which can
                              // be used to create an array or so
};


} // namespace cura
