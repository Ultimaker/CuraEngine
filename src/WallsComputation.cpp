/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "WallsComputation.h"
#include "utils/polygonUtils.h"
namespace cura {

WallsComputation::WallsComputation(int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool recompute_outline_based_on_outer_wall)
: wall_0_inset(wall_0_inset)
, line_width_0(line_width_0)
, line_width_x(line_width_x)
, insetCount(insetCount)
, recompute_outline_based_on_outer_wall(recompute_outline_based_on_outer_wall)
{
}

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
        } else if (i == 1)
        {
            part->insets[1] = part->insets[0].offset(-line_width_0 / 2 + wall_0_inset - line_width_x / 2);
        } else
        {
            part->insets[i] = part->insets[i-1].offset(-line_width_x);
        }
        
        
        //Finally optimize all the polygons. Every point removed saves time in the long run.
        part->insets[i].simplify();
        if (i == 0)
        {
            if (recompute_outline_based_on_outer_wall)
            {
                part->print_outline = part->insets[0].offset(line_width_0 / 2);
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

void WallsComputation::generateInsets(SliceLayer* layer)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr]);
    }
    
    //Remove the parts which did not generate an inset. As these parts are too small to print,
    // and later code can now assume that there is always minimal 1 inset line.
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        if (layer->parts[partNr].insets.size() < 1)
        {
            layer->parts.erase(layer->parts.begin() + partNr);
            partNr -= 1;
        }
    }
}

}//namespace cura
