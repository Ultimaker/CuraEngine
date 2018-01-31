/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#include "WallsComputation.h"
#include "utils/polygonUtils.h"
namespace cura {

WallsComputation::WallsComputation(int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool recompute_outline_based_on_outer_wall, bool remove_parts_with_no_insets)
: wall_0_inset(wall_0_inset)
, line_width_0(line_width_0)
, line_width_x(line_width_x)
, insetCount(insetCount)
, recompute_outline_based_on_outer_wall(recompute_outline_based_on_outer_wall)
, remove_parts_with_no_insets(remove_parts_with_no_insets)
{
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateInsets only reads and writes data for the current layer
 */
void WallsComputation::generateInsets(SliceLayerPart* part)
{
    const float undersize_factor = 0.95f; // allow innermost inset to be this much narrower than normal
    if (insetCount == 0)
    {
        part->insets.push_back(part->outline);
        part->print_outline = part->outline;
        return;
    }

    for(int i=0; i<insetCount; i++)
    {
        part->insets.push_back(Polygons());
        if (i == 0)
        {
            part->insets[0] = part->outline.offset(-line_width_0 / 2 - wall_0_inset);
        } else if (i == 1)
        {
            const int offset = -line_width_0 / 2 + wall_0_inset - line_width_x / 2;
            part->insets[1] = part->insets[0].offset(offset);
            if (part->insets[1].size() == 0)
            {
                // if a slightly thinner line would fit, allow it as the overlap is so small
                // this helps when you have parts such as tubes whose walls are an exact number of line widths wide
                // e.g. when the tube wall is 1.5mm wide and the line widths are 0.5mm
                part->insets[1] = part->insets[0].offset(offset * undersize_factor);
            }
        } else
        {
            part->insets[i] = part->insets[i-1].offset(-line_width_x);
            if (part->insets[i].size() == 0)
            {
                // if a slightly thinner line would fit, allow it as the overlap is so small
                part->insets[i] = part->insets[i-1].offset(-line_width_x * undersize_factor);
            }
        }

        //Finally optimize all the polygons. Every point removed saves time in the long run.
        part->insets[i].simplify();
        part->insets[i].removeDegenerateVerts();
        if (i == 0)
        {
            if (recompute_outline_based_on_outer_wall)
            {
                part->print_outline = part->insets[0].offset(line_width_0 / 2, ClipperLib::jtSquare);
            }
            else
            {
                part->print_outline = part->outline;
            }
        }
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateInsets only reads and writes data for the current layer
 */
void WallsComputation::generateInsets(SliceLayer* layer)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr]);
    }

    //Remove the parts which did not generate an inset. As these parts are too small to print,
    // and later code can now assume that there is always minimal 1 inset line.
    for (unsigned int part_idx = 0; part_idx < layer->parts.size(); part_idx++)
    {
        if (layer->parts[part_idx].insets.size() == 0 && remove_parts_with_no_insets)
        {
            if (part_idx != layer->parts.size() - 1)
            { // move existing part into part to be deleted
                layer->parts[part_idx] = std::move(layer->parts.back());
            }
            layer->parts.pop_back(); // always remove last element from array (is more efficient)
            part_idx -= 1; // check the part we just moved here
        }
    }
}

}//namespace cura
