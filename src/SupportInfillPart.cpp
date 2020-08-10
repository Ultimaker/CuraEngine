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

void SupportInfillPart::generateInsets()
{
    // generate insets, use the first inset as the wall line, and the second as the infill area
    AreaSupport::generateSupportWalls(wall_toolpaths, wall_area, support_line_width);
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
        constexpr coord_t smallest_line_segment = 50; // Todo: get rid of magic number
        constexpr coord_t allowed_error_distance = 50; // Todo: get rid of magic number
        const auto half_wall = support_line_width / 2;
        const auto inner_offset = -half_wall - support_line_width * (inset_count_to_generate - 1);
        infill_area = outline.offset(inner_offset);
        infill_area.simplify(smallest_line_segment, allowed_error_distance);
        auto outer_inset = outline.offset(half_wall);
        outer_inset.simplify();
        wall_area = outer_inset.difference(infill_area);
        wall_area.removeColinearEdges(0.03);
        wall_area.fixSelfIntersections();
        const double small_area = static_cast<double>(support_line_width * support_line_width) /
                                  4e6; // same as (wall_line_width_x / 2 / 1000)**2
        wall_area.removeSmallAreas(small_area, false);
    }
}
