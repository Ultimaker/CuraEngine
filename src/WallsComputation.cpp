//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "WallsComputation.h"
#include "utils/polygonUtils.h"
namespace cura {

WallsComputation::WallsComputation(int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool recompute_outline_based_on_outer_wall, bool remove_parts_with_no_insets, const bool try_line_thickness)
: wall_0_inset(wall_0_inset)
, line_width_0(line_width_0)
, line_width_x(line_width_x)
, insetCount(insetCount)
, recompute_outline_based_on_outer_wall(recompute_outline_based_on_outer_wall)
, remove_parts_with_no_insets(remove_parts_with_no_insets)
, try_line_thickness(try_line_thickness)
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
        }
        else if (i == 1)
        {
            part->insets[1] = part->insets[0].offset(-line_width_0 / 2 + wall_0_inset - line_width_x / 2);
        }
        else
        {
            part->insets[i] = part->insets[i - 1].offset(-line_width_x);
        }

        const size_t inset_part_count = part->insets[i].size();
        constexpr size_t minimum_part_saving = 3; //Only try if the part has more pieces than the previous inset and saves at least this many parts.
        constexpr coord_t try_smaller = 10; //How many micrometres to inset with the try with a smaller inset.
        if (try_line_thickness && inset_part_count > minimum_part_saving + 1 && (i == 0 || (i > 0 && inset_part_count > part->insets[i - 1].size() + minimum_part_saving)))
        {
            //Try a different line thickness and see if this fits better, based on these criteria:
            // - There are fewer parts to the polygon (fits better in slim areas).
            // - The polygon area is largely unaffected.
            Polygons alternative_inset;
            if (i == 0)
            {
                alternative_inset = part->outline.offset(-(line_width_0 - try_smaller) / 2 - wall_0_inset);
            }
            else if (i == 1)
            {
                alternative_inset = part->outline.offset(-(line_width_0 - try_smaller) / 2 + wall_0_inset - line_width_x / 2);
            }
            else
            {
                alternative_inset = part->insets[i - 1].offset(-(line_width_x - try_smaller));
            }
            if (alternative_inset.size() < inset_part_count - minimum_part_saving) //Significantly fewer parts (saves more than 3 parts).
            {
                part->insets[i] = alternative_inset;
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
