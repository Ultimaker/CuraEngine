// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_SECTION_TYPE_H
#define INCLUDE_UTILS_SECTION_TYPE_H

namespace cura
{
enum class SectionType : int
{
    NA = -1,
    WALL = 1,
    INFILL = 2,
    SKIN = 3,
    SUPPORT = 4,
    ADHESION = 5,
    IRONING = 6,
    MESH = 7,
    DOTS = 8,
    CONCENTRIC_INFILL = 9
};
} // namespace cura

#endif // INCLUDE_UTILS_SECTION_TYPE_H
