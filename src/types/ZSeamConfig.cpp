// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/ZSeamConfig.h" //The definitions we're implementing.

using namespace curaengine::geometry;
using namespace curaengine::utils;

namespace curaengine::types
{

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point2LL pos, const EZSeamCornerPrefType corner_pref, const coord_t simplify_curvature)
    : type_(type)
    , pos_(pos)
    , corner_pref_(corner_pref)
    , simplify_curvature_(simplify_curvature)
{
}

} // namespace curaengine::types
