// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"

#include "utils/area_support.h"

namespace curaengine
{
using namespace types::geometry;
using namespace types;
using namespace utils;


SupportInfillPart::SupportInfillPart(
    const SingleShape& outline,
    coord_t support_line_width,
    bool use_fractional_config,
    coord_t inset_width_to_generate,
    coord_t custom_line_distance)
    : outline_(outline)
    , outline_boundary_box_(outline)
    , support_line_width_(support_line_width)
    , inset_width_to_generate_(inset_width_to_generate)
    , custom_line_distance_(custom_line_distance)
    , use_fractional_config_(use_fractional_config)
{
}

} // namespace curaengine
