/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "inset.h"
#include "polygonOptimizer.h"

namespace cura {

void generateInsets(SliceLayerPart* part, int offset, int bulgeOffset, int insetCount)
{
    part->combBoundery = part->outline.offset(-offset-bulgeOffset);
    if (insetCount == 0)
    {
        part->insets.push_back(part->outline);
        return;
    }
    
    for(int i=0; i<insetCount; i++)
    {
        part->insets.push_back(Polygons());
        part->insets[i] = part->outline.offset(-offset * i - offset/2 - bulgeOffset);
        optimizePolygons(part->insets[i]);
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}

void generateInsets(SliceLayer* layer, int offset, int bulgeOffset, int insetCount)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], offset, bulgeOffset, insetCount);
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
