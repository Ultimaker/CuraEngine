/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skin.h"

namespace cura {

void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int infillOverlap)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];
        
        Polygons upskin = part->insets[part->insets.size() - 1].offset(-extrusionWidth/2);
        Polygons downskin = upskin;
        
        if (part->insets.size() > 1)
        {
            //Add thin wall filling by taking the area between the insets.
            Polygons thinWalls = part->insets[0].offset(-extrusionWidth / 2 - extrusionWidth * infillOverlap / 100).difference(part->insets[1].offset(extrusionWidth * 6 / 10));
            upskin.add(thinWalls);
            downskin.add(thinWalls);
        }
        if (static_cast<int>(layerNr - downSkinCount) >= 0)
        {
            SliceLayer* layer2 = &storage.layers[layerNr - downSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                    downskin = downskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1]);
            }
        }
        if (static_cast<int>(layerNr + upSkinCount) < static_cast<int>(storage.layers.size()))
        {
            SliceLayer* layer2 = &storage.layers[layerNr + upSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                    upskin = upskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1]);
            }
        }
        
        part->skinOutline = upskin.unionPolygons(downskin);

        double minAreaSize = (2 * M_PI * INT2MM(extrusionWidth) * INT2MM(extrusionWidth)) * 0.3;
        for(unsigned int i=0; i<part->skinOutline.size(); i++)
        {
            double area = INT2MM(INT2MM(fabs(part->skinOutline[i].area())));
            if (area < minAreaSize) // Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill"
            {
                part->skinOutline.remove(i);
                i -= 1;
            }
        }
    }
}

void generateSparse(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];

        Polygons sparse = part->insets[part->insets.size() - 1].offset(-extrusionWidth/2);
        Polygons downskin = sparse;
        Polygons upskin = sparse;
        
        if (static_cast<int>(layerNr - downSkinCount) >= 0)
        {
            SliceLayer* layer2 = &storage.layers[layerNr - downSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                {
                    if (layer2->parts[partNr2].insets.size() > 1)
                    {
                        downskin = downskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 2]);
                    }else{
                        downskin = downskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1]);
                    }
                }
            }
        }
        if (static_cast<int>(layerNr + upSkinCount) < static_cast<int>(storage.layers.size()))
        {
            SliceLayer* layer2 = &storage.layers[layerNr + upSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                {
                    if (layer2->parts[partNr2].insets.size() > 1)
                    {
                        upskin = upskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 2]);
                    }else{
                        upskin = upskin.difference(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1]);
                    }
                }
            }
        }
        
        Polygons result = upskin.unionPolygons(downskin);

        double minAreaSize = 3.0;//(2 * M_PI * INT2MM(config.extrusionWidth) * INT2MM(config.extrusionWidth)) * 3;
        for(unsigned int i=0; i<result.size(); i++)
        {
            double area = INT2MM(INT2MM(fabs(result[i].area())));
            if (area < minAreaSize) /* Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill" */
            {
                result.remove(i);
                i -= 1;
            }
        }
        
        part->sparse_outline.push_back(sparse.difference(result));
    }
}

void combineSparseLayers(int layerNr, SliceMeshStorage& storage, int amount)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(int n=1; n<amount; n++)
    {
        if (layerNr < n)
            break;
        
        SliceLayer* layer2 = &storage.layers[layerNr - n];
        for(SliceLayerPart& part : layer->parts)
        {
            Polygons result;
            for(SliceLayerPart& part2 : layer2->parts)
            {
                if (part.boundaryBox.hit(part2.boundaryBox))
                {
                    Polygons intersection = part.sparse_outline[n - 1].intersection(part2.sparse_outline[0]).offset(-200).offset(200);
                    result.add(intersection);
                    part.sparse_outline[n - 1] = part.sparse_outline[n - 1].difference(intersection);
                    part2.sparse_outline[0] = part2.sparse_outline[0].difference(intersection);
                }
            }
            
            part.sparse_outline.push_back(result);
        }
    }
}

}//namespace cura
