//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "settings/ZSeamConfig.h" //The definitions we're implementing.

namespace cura
{

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref, const coord_t simplify_curvature)
: type(type)
, pos(pos)
, corner_pref(corner_pref)
, simplify_curvature(simplify_curvature)
{
}

} //Cura namespace.