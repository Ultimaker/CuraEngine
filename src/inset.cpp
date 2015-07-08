/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "inset.h"
#include "polygonOptimizer.h"
#include "utils/polygonUtils.h"
namespace cura {

void generateInsets(SliceLayerPart* part, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
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
            offsetSafe(part->outline, - line_width_x/2, line_width_x, part->insets[i], avoidOverlappingPerimeters_0);
        } else if (i == 1)
        {
            offsetExtrusionWidth(part->insets[i-1], true, line_width_0, part->insets[i], &part->perimeterGaps, avoidOverlappingPerimeters);
        } else
        {
            offsetExtrusionWidth(part->insets[i-1], true, line_width_x, part->insets[i], &part->perimeterGaps, avoidOverlappingPerimeters);
        }
            
        optimizePolygons(part->insets[i]);
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}


void generateInsets(SliceLayer* layer, int line_width_0, int line_width_x, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], line_width_0, line_width_x, insetCount, avoidOverlappingPerimeters_0, avoidOverlappingPerimeters);
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
