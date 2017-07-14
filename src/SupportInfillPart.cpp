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
    infill_area.clear();
    infill_area_per_combine_per_density.clear();
}


SupportInfillPart::~SupportInfillPart()
{
    support_line_width = 0;
    inset_count_to_generate = 0;
    outline.clear();
    insets.clear();
    infill_area.clear();
    infill_area_per_combine_per_density.clear();
}


bool SupportInfillPart::generateInsetsAndInfillAreas()
{
    // generate insets, use the first inset as the wall line, and the second as the infill area
    AreaSupport::generateOutlineInsets(
        this->insets,
        this->outline,
        this->inset_count_to_generate,
        this->support_line_width);
    if (this->inset_count_to_generate > 0 && this->insets.empty())
    {
        return false;
    }

    // calculate infill area if there are insets
    if (this->inset_count_to_generate > 0)
    {
        // if there are walls, we use the inner area as the infill area
        this->infill_area = this->insets.back().offset(-support_line_width / 2);
        // optimize polygons: remove unnecessary verts
        this->infill_area.simplify();
    }

    return true;
}
