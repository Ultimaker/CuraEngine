#ifndef INSET_H
#define INSET_H

class InsetPart
{
public:
    std::vector<ClipperLib::Polygons> inset;
    
    InsetPart(LayerPart* part, int offset, int insetCount)
    {
        for(int i=0; i<insetCount; i++)
        {
            inset.push_back(ClipperLib::Polygons());
            ClipperLib::OffsetPolygons(part->polygons, inset[i], -offset * i - offset/2, ClipperLib::jtSquare, 2, false);
            if (inset[i].size() < 1)
                break;
        }
    }
};

class InsetLayer
{
public:
    std::vector<InsetPart> parts;

    InsetLayer(LayerPartsLayer* layer, int offset, int insetCount)
    {
        for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
        {
            parts.push_back(InsetPart(&layer->parts[partNr], offset, insetCount));
        }
    }
    
    ~InsetLayer()
    {
    }
};

#endif//INSET_H
