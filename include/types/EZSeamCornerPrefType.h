// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
enum class EZSeamCornerPrefType
{
    Z_SEAM_CORNER_PREF_INNER,
    Z_SEAM_CORNER_PREF_OUTER,
    Z_SEAM_CORNER_PREF_ANY,
    Z_SEAM_CORNER_PREF_WEIGHTED,
    PLUGIN,
};

} // namespace curaengine::types
