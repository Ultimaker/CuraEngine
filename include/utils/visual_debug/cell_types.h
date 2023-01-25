// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_CELL_TYPES_H
#define INCLUDE_UTILS_VISUAL_DEBUG_CELL_TYPES_H

namespace cura::debug
{

enum class CellTypes : long
{
    VERTEX = 1,
    POLY_VERTEX = 2,
    LINE = 3,
    POLY_LINE = 4,
    TRIANGLE = 5,
    TRIANGLE_STRIP = 6,
    POLYGON = 7,
    PIXEL = 8,
    QUAD = 9,
    TETRA = 10,
    VOXEL = 11,
    HEXAHEDRON = 12,
    WEDGE = 13,
    PYRAMID = 14
};
} // namespace cura::debug

#endif //INCLUDE_UTILS_VISUAL_DEBUG_CELL_TYPES_H
