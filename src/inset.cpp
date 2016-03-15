/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "inset.h"
#include "utils/polygonUtils.h"
namespace cura {

void generateInsets(SliceLayerPart* part, int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
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
            PolygonUtils::offsetSafe(part->outline, -line_width_0 / 2 - wall_0_inset, line_width_0, part->insets[0], avoidOverlappingPerimeters_0);
        } else if (i == 1)
        {
            int offset_from_first_boundary_for_edge_of_outer_wall = -line_width_0 / 2; // the outer bounds of the perimeter gaps
            // you might think this /\ should be (1): -line_width_0 / 2; or you might think it should be (2): -nozzle_size / 2
            // (1): the volume extruded is the right volume; the infill gaps overlap more with the outer wall
            // (2): the outer wall already fills up extra space due to the fact that the nozzle hole overlaps with a part inside the outer wall
            PolygonUtils::offsetSafe(part->insets[0], -line_width_0 / 2 + wall_0_inset - line_width_x / 2, offset_from_first_boundary_for_edge_of_outer_wall, line_width_x, part->insets[1], &part->perimeterGaps, avoidOverlappingPerimeters);
        } else
        {
            PolygonUtils::offsetExtrusionWidth(part->insets[i-1], true, line_width_x, part->insets[i], &part->perimeterGaps, avoidOverlappingPerimeters);
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

void generateInsets(SliceLayer* layer, int wall_0_inset, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], wall_0_inset, line_width_0, line_width_x, insetCount, avoidOverlappingPerimeters_0, avoidOverlappingPerimeters);
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
