// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"

using namespace cura;


SupportInfillPart::SupportInfillPart(const PolygonsPart& outline, coord_t support_line_width, int inset_count_to_generate)
: outline(outline)
, outline_boundary_box(outline)
, support_line_width(support_line_width)
, inset_count_to_generate(inset_count_to_generate)
{
    infill_area_per_combine_per_density.clear();
}
