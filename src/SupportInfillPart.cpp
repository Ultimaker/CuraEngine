// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"

#include "support.h"

using namespace cura;


SupportInfillPart::SupportInfillPart(const SingleShape& outline, coord_t support_line_width, bool use_fractional_config,  int inset_count_to_generate, coord_t custom_line_distance, EFillMethod custom_line_pattern,  std::optional<Point2LL> start_near_location)
    : outline_(outline)
    , outline_boundary_box_(outline)
    , support_line_width_(support_line_width)
    , inset_count_to_generate_(inset_count_to_generate)
    , custom_line_distance_(custom_line_distance)
    , custom_line_pattern_(custom_line_pattern)
    , use_fractional_config_(use_fractional_config)
    , start_near_location(start_near_location)

{
    infill_area_per_combine_per_density_.clear();
}
