/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "inset.h"
#include "polygonOptimizer.h"
#include "utils/polygonUtils.h"
namespace cura {

void generateInsets(SliceLayerPart* part, int nozzle_width, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    if (insetCount == 0)
    {
        part->insets.push_back(part->outline);
        return;
    }
    
    for(int i=0; i<insetCount; i++)
    {
        part->insets.push_back(Polygons());
        if (i == 0)
        {
            if (line_width_0 < nozzle_width)
            {
                offsetSafe(part->outline, - nozzle_width/2, line_width_0, part->insets[0], avoidOverlappingPerimeters_0);
            }
            else 
            {
                offsetSafe(part->outline, - line_width_0/2, line_width_0, part->insets[0], avoidOverlappingPerimeters_0);
            }
        } else if (i == 1)
        {
            if (line_width_0 < nozzle_width)
            {
                int offset_from_first_boundary_for_edge_of_outer_wall = -nozzle_width/2; 
                // ideally this /\ should be: nozzle_width/2 - line_width_0; however, factually, the nozzle will fill up part of the perimeter gaps
                offsetSafe(part->insets[0], nozzle_width/2 - line_width_0 - line_width_x/2, offset_from_first_boundary_for_edge_of_outer_wall, line_width_x, part->insets[1], &part->perimeterGaps, avoidOverlappingPerimeters);
            }
            else 
            {
                offsetSafe(part->insets[0], -line_width_0/2 - line_width_x/2, -line_width_0/2, line_width_x, part->insets[1], &part->perimeterGaps, avoidOverlappingPerimeters);
            }
        } else
        {
            offsetExtrusionWidth(part->insets[i-1], true, line_width_x, part->insets[i], &part->perimeterGaps, avoidOverlappingPerimeters);
        }
        
        
        //Finally optimize all the polygons. Every point removed saves time in the long run.
        part->insets[i].simplify();
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}

void generateInsets(SliceLayer* layer, int nozzle_width, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], nozzle_width, line_width_0, line_width_x, insetCount, avoidOverlappingPerimeters_0, avoidOverlappingPerimeters);
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
