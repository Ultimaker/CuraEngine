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

bool SupportInfillPart::generateInsets()
{
    // generate insets, use the first inset as the wall line, and the second as the infill area
    AreaSupport::generateSupportWalls(wall_toolpaths, outline, inset_count_to_generate, support_line_width);
    return (inset_count_to_generate > 0 && !wall_toolpaths.empty());
}

void SupportInfillPart::generateInfillAreas()
{
    // if there are walls, we use the inner area as the infill area
    const coord_t inner_offset = support_line_width * (inset_count_to_generate - 1) + support_line_width;
    const double small_area = static_cast<double>(support_line_width * support_line_width) / 4e6; // same as (wall_line_width_x / 2 / 1000)**2
    infill_area = outline.offset(-inner_offset); // Todo get outline from last inner_inset, don't recalculate
    AreaSupport::prepareInsetForToolpathGeneration(infill_area, small_area);
}

bool SupportInfillPart::generateInsetsAndInfillAreas()
{
    // generate insets, use the first inset as the wall line, and the second as the infill area
    AreaSupport::generateOutlineInsets(insets, outline, inset_count_to_generate, support_line_width);
    if (inset_count_to_generate > 0 && insets.empty())
    {
        return false;
    }

    // calculate infill area if there are insets
    if (inset_count_to_generate > 0)
    {
        // if there are walls, we use the inner area as the infill area
        infill_area = insets.back().offset(-support_line_width / 2);
        // optimize polygons: remove unnecessary verts
        infill_area.simplify();
    }

    return true;
}
