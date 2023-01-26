// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_SECTION_TYPE_H
#define INCLUDE_UTILS_VISUAL_DEBUG_SECTION_TYPE_H

namespace cura::debug
{
enum class SectionType : long
{
    NA = -1,
    WALL = 0,
    INFILL = 1,
    SKIN = 2,
    SUPPORT = 3,
    ADHESION = 4
};
} // namespace cura::debug

#endif //INCLUDE_UTILS_VISUAL_DEBUG_SECTION_TYPE_H
