//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "WallsComputation.h"
#include "settings/types/Ratio.h"
#include "settings/EnumSettings.h"
#include "utils/polygonUtils.h"
#include "Application.h"
#include "Slice.h"

namespace cura {

WallsComputation::WallsComputation(const Settings& settings, const LayerIndex layer_nr)
: settings(settings)
, layer_nr(layer_nr)
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
    size_t inset_count = settings.get<size_t>("wall_line_count");
    const bool spiralize = settings.get<bool>("magic_spiralize");
    if (spiralize && layer_nr < LayerIndex(settings.get<size_t>("initial_bottom_layers")) && ((layer_nr % 2) + 2) % 2 == 1) //Add extra insets every 2 layers when spiralizing. This makes bottoms of cups watertight.
    {
        inset_count += 5;
    }
    if (settings.get<bool>("alternate_extra_perimeter"))
    {
        inset_count += ((layer_nr % 2) + 2) % 2;
    }

    if (inset_count == 0)
    {
        part->insets.push_back(part->outline);
        part->print_outline = part->outline;
        return;
    }

    const coord_t wall_0_inset = settings.get<coord_t>("wall_0_inset");
    coord_t line_width_0 = settings.get<coord_t>("wall_line_width_0");
    coord_t line_width_x = settings.get<coord_t>("wall_line_width_x");
    if (layer_nr == 0)
    {
        const ExtruderTrain& train_wall_0 = settings.get<ExtruderTrain&>("wall_0_extruder_nr");
        line_width_0 *= train_wall_0.settings.get<Ratio>("initial_layer_line_width_factor");
        const ExtruderTrain& train_wall_x = settings.get<ExtruderTrain&>("wall_x_extruder_nr");
        line_width_x *= train_wall_x.settings.get<Ratio>("initial_layer_line_width_factor");
    }

    const bool recompute_outline_based_on_outer_wall =
        settings.get<bool>("support_enable") &&
        !settings.get<bool>("fill_outline_gaps");
    for(size_t i = 0; i < inset_count; i++)
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
        if (inset_part_count > minimum_part_saving + 1 && (i == 0 || (i > 0 && inset_part_count > part->insets[i - 1].size() + minimum_part_saving)))
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
                alternative_inset = part->insets[0].offset(-(line_width_0 - try_smaller) / 2 + wall_0_inset - line_width_x / 2);
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
        int wall_extruder_nr = settings.get<int>(i == 0 ? "wall_0_extruder_nr" : "wall_x_extruder_nr");
        const Settings& resolution_settings = wall_extruder_nr < 0 ? settings : Application::getInstance().current_slice->scene.extruders[wall_extruder_nr].settings;
        const coord_t maximum_resolution = resolution_settings.get<coord_t>("meshfix_maximum_resolution");
        const coord_t maximum_deviation = resolution_settings.get<coord_t>("meshfix_maximum_deviation");
        part->insets[i].simplify(maximum_resolution, maximum_deviation);
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

    const bool remove_parts_with_no_insets = !settings.get<bool>("fill_outline_gaps");
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
