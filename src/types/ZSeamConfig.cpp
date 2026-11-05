// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/ZSeamConfig.h"

namespace curaengine::types
{
using namespace types::geometry;
using namespace utils;

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point2LL pos, const EZSeamCornerPrefType corner_pref, const coord_t simplify_curvature)
    : type_(type)
    , pos_(pos)
    , corner_pref_(corner_pref)
    , simplify_curvature_(simplify_curvature)
{
}

} // namespace curaengine::types
