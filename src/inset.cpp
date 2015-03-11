/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "inset.h"
#include "polygonOptimizer.h"

namespace cura {

void generateInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters)
{
    part->combBoundery = part->outline.offset(-extrusionWidth);
    if (insetCount == 0)
    {
        part->insets.push_back(part->outline);
        return;
    }
    
    for(int i=0; i<insetCount; i++)
    {
        part->insets.push_back(Polygons());
        if (avoidOverlappingPerimeters && i > 0)
        {
            Polygons inner_bounds = part->insets[i-1].offset(-extrusionWidth - extrusionWidth / 2);
            Polygons nonOverlapping = inner_bounds.offset(extrusionWidth / 2);
            Polygons simple_inset = part->insets[i-1].offset(-extrusionWidth);
            part->insets[i] = nonOverlapping.intersection(simple_inset);
        } else
            part->insets[i] = part->outline.offset(-extrusionWidth * i - extrusionWidth/2);
            
        optimizePolygons(part->insets[i]);
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}

void generateInsets(SliceLayer* layer, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], extrusionWidth, insetCount, avoidOverlappingPerimeters);
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
