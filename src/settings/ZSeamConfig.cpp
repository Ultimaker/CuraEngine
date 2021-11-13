//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ZSeamConfig.h" //The definitions we're implementing.

namespace cura
{

ZSeamConfig::ZSeamConfig()
    : type(EZSeamType::SHORTEST)
    , pos(Point(0, 0))
    , corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
{}

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref)
    : type(type)
    , pos(pos)
    , corner_pref(corner_pref)
{}

}