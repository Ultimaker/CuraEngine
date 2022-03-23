//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "WallsComputation.h"
#include "settings/types/Ratio.h"
#include "WallToolPaths.h"
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
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayerPart* part)
{
    size_t wall_count = settings.get<size_t>("wall_line_count");
    if (part->roofing_wall || layer_nr == 0)
    {
        wall_count  = 1;
    }
    if (wall_count == 0) // Early out if no walls are to be generated
    {
        part->print_outline = part->outline;
        part->inner_area = part->outline;
        return;
    }
    const bool spiralize = settings.get<bool>("magic_spiralize");
    const size_t alternate = ((layer_nr % 2) + 2) % 2;
    if (spiralize && layer_nr < LayerIndex(settings.get<size_t>("initial_bottom_layers")) && alternate == 1) //Add extra insets every 2 layers when spiralizing. This makes bottoms of cups watertight.
    {
        wall_count += 5;
    }
    if (settings.get<bool>("alternate_extra_perimeter"))
    {
        wall_count += alternate;
    }

    const bool first_layer = layer_nr == 0;
    const Ratio line_width_0_factor = first_layer ? settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_0 =  first_layer ? settings.get<coord_t>("skin_line_width") * line_width_0_factor : settings.get<coord_t>("wall_line_width_0");
    const coord_t wall_0_inset = settings.get<coord_t>("wall_0_inset");

    const Ratio line_width_x_factor = first_layer ? settings.get<ExtruderTrain&>("wall_x_extruder_nr").settings.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_x = first_layer ? settings.get<coord_t>("skin_line_width") * line_width_x_factor : settings.get<coord_t>("wall_line_width_x");

    // When spiralizing, generate the spiral insets using simple offsets instead of generating toolpaths
    if (spiralize)
    {
        const bool recompute_outline_based_on_outer_wall =
          settings.get<bool>("support_enable") && !settings.get<bool>("fill_outline_gaps");

        generateSpiralInsets(part, line_width_0, wall_0_inset, recompute_outline_based_on_outer_wall);
        if (layer_nr <= static_cast<LayerIndex>(settings.get<size_t>("bottom_layers")))
        {
            WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, settings);
            part->wall_toolpaths = wall_tool_paths.getToolPaths();
            part->inner_area = wall_tool_paths.getInnerContour();
        }
    }
    else
    {
        WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, settings);
        part->wall_toolpaths = wall_tool_paths.getToolPaths();
        part->inner_area = wall_tool_paths.getInnerContour();
    }
    part->print_outline = part->outline;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayer* layer)
{
    for(SliceLayerPart& part : layer->parts)
    {
        generateWalls(&part);
    }

    //Remove the parts which did not generate a wall. As these parts are too small to print,
    // and later code can now assume that there is always minimal 1 wall line.
    if(settings.get<size_t>("wall_line_count") >= 1 && !settings.get<bool>("fill_outline_gaps"))
    {
        for(size_t part_idx = 0; part_idx < layer->parts.size(); part_idx++)
        {
            if (layer->parts[part_idx].wall_toolpaths.empty() && layer->parts[part_idx].spiral_wall.empty())
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
}

void WallsComputation::generateSpiralInsets(SliceLayerPart *part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall)
{
    part->spiral_wall = part->outline.offset(-line_width_0 / 2 - wall_0_inset);

    //Optimize the wall. This prevents buffer underruns in the printer firmware, and reduces processing time in CuraEngine.
    const ExtruderTrain& train_wall = settings.get<ExtruderTrain&>("wall_0_extruder_nr");
    const coord_t maximum_resolution = train_wall.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t maximum_deviation = train_wall.settings.get<coord_t>("meshfix_maximum_deviation");
    part->spiral_wall.simplify(maximum_resolution, maximum_deviation);
    part->spiral_wall.removeDegenerateVerts();
    if (recompute_outline_based_on_outer_wall)
    {
        part->print_outline = part->spiral_wall.offset(line_width_0 / 2, ClipperLib::jtSquare);
    }
    else
    {
        part->print_outline = part->outline;
    }
}

}//namespace cura