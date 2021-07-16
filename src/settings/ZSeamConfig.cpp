//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ZSeamConfig.h" //The definitions we're implementing.

namespace cura
{

ZSeamConfig::ZSeamConfig()
: type(EZSeamType::SHORTEST)
, pos(Point(0, 0))
, corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
, simplify_curvature(0)
{
}

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref, const coord_t simplify_curvature)
: type(type)
, pos(pos)
, corner_pref(corner_pref)
, simplify_curvature(simplify_curvature)
{
}

} //Cura namespace.