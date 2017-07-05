// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportInfillPart.h"
#include "support.h"

using namespace cura;


SupportInfillPart::SupportInfillPart(coord_t support_line_width, int infill_overlap, int inset_count_to_generate)
    : support_line_width(support_line_width)
    , infill_overlap(infill_overlap)
    , inset_count_to_generate(inset_count_to_generate)
{
    outline.clear();
    insets.clear();
    infill_area.clear();
    infill_areas_per_combine_per_density.clear();
}


SupportInfillPart::~SupportInfillPart()
{
    support_line_width = 0;
    infill_overlap = 0;
    inset_count_to_generate = 0;
    outline.clear();
    insets.clear();
    infill_area.clear();
    infill_areas_per_combine_per_density.clear();
}


bool SupportInfillPart::initializeWithOutline(const Polygons& outline)
{
    this->outline = outline;

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

    // get infill area
    if (this->inset_count_to_generate == 0)
    {
        // if there is no wall, we use the original outline as the infill area
        this->infill_area = this->outline;
    }
    else
    {
        // if there are walls, we use the inner area as the infill area
        this->infill_area = this->insets.back().offset(-support_line_width / 2);
        if (!this->infill_area.empty())
        {
            // optimize polygons: remove unnecessary verts
            this->infill_area.simplify();
        }
    }
    // also create the boundary box using the outline
    this->outline_boundary_box = AABB(this->outline);

    return true;
}


void SupportInfillPart::splitIntoSmallerParts(std::vector<SupportInfillPart> smaller_parts, Polygons excluding_areas)
{
    smaller_parts.clear();

    Polygons result_polygons = outline.difference(excluding_areas);
    std::vector<PolygonsPart> support_islands = result_polygons.splitIntoParts();

    for (const PolygonsPart& island_outline : support_islands)
    {
        if (island_outline.empty())
        {
            continue;
        }

        // create a new part
        SupportInfillPart support_infill_part(this->support_line_width, this->infill_overlap, this->inset_count_to_generate);
        if (support_infill_part.initializeWithOutline(island_outline))
        {
            continue;
        }

        smaller_parts.push_back(support_infill_part);
    }
}
