// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"

#include "support.h"

using namespace cura;


SupportInfillPart::SupportInfillPart(const PolygonsPart& outline, coord_t support_line_width, bool use_fractional_config, int inset_count_to_generate, coord_t custom_line_distance)
    : outline(outline)
    , outline_boundary_box(outline)
    , support_line_width(support_line_width)
    , inset_count_to_generate(inset_count_to_generate)
    , custom_line_distance(custom_line_distance)
    , use_fractional_config(use_fractional_config)
{
    infill_area_per_combine_per_density.clear();
}
