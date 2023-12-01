// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "settings/ZSeamConfig.h" //The definitions we're implementing.

namespace cura
{

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point2LL pos, const EZSeamCornerPrefType corner_pref, const coord_t simplify_curvature)
    : type_(type)
    , pos_(pos)
    , corner_pref_(corner_pref)
    , simplify_curvature_(simplify_curvature)
{
}

} // namespace cura
