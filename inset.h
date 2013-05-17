/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INSET_H
#define INSET_H

void generateInsets(SliceLayerPart* part, int offset, int insetCount)
{
    for(int i=0; i<insetCount; i++)
    {
        part->insets.push_back(Polygons());
        ClipperLib::OffsetPolygons(part->outline, part->insets[i], -offset * i - offset/2, ClipperLib::jtSquare, 2, false);
        optimizePolygons(part->insets[i]);
        if (part->insets[i].size() < 1)
        {
            part->insets.pop_back();
            break;
        }
    }
}

void generateInsets(SliceLayer* layer, int offset, int insetCount)
{
    for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
    {
        generateInsets(&layer->parts[partNr], offset, insetCount);
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

#endif//INSET_H
