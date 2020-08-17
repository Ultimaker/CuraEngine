// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"
#include "support.h"

using namespace cura;


SupportInfillPart::SupportInfillPart(const PolygonsPart& outline, coord_t support_line_width, int inset_count_to_generate)
: outline(outline)
, outline_boundary_box(outline)
, support_line_width(support_line_width)
, inset_count_to_generate(inset_count_to_generate)
{
    insets.clear();
    wall_toolpaths.clear();
    infill_area.clear();
    infill_area_per_combine_per_density.clear();
}

void SupportInfillPart::generateInsetsAndInfillAreas()
{
    if (inset_count_to_generate == 0)
    {
        infill_area = outline;
        infill_area.simplify();
    }
    else
    {
        constexpr coord_t smallest_segment = 50;
        constexpr coord_t allowed_distance = 50;
        constexpr coord_t epsilon_offset = 10;
        constexpr float max_colinear_angle = 0.03;  // Way too large   TODO: after we ironed out all the bugs, remove-colinear should go.
        constexpr bool remove_holes = false;
        const double small_area_length = INT2MM(static_cast<double>(support_line_width) / 2);

        prepared_outline = outline.offset(-epsilon_offset).offset(epsilon_offset);
        prepared_outline.simplify(smallest_segment, allowed_distance);
        prepared_outline.removeColinearEdges(max_colinear_angle);
        prepared_outline.fixSelfIntersections();
        prepared_outline.removeSmallAreas(small_area_length * small_area_length, remove_holes); // TODO: complete guess as to when arachne starts breaking, but it doesn't function well when an area is really small apearantly?
        infill_area = outline.offset(-static_cast<coord_t>(small_area_length) - static_cast<coord_t>(support_line_width) * (inset_count_to_generate)); // Todo get infill area from generated walls see CURA-7653
        infill_area.simplify();
    }
}
