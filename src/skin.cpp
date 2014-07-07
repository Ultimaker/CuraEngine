/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skin.h"

namespace cura {




void generateSparseAndSkins(int layerNr, SliceVolumeStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int infillOverlap)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];

        Polygons sparse = part->insets[part->insets.size() - 1].offset(-extrusionWidth/2);
        Polygons downskin = sparse;
        Polygons upskin = sparse;

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

        //part->skinOutline = upskin.unionPolygons(downskin);
        SkinPart skinPart;
        skinPart.outline = upskin;
        skinPart.skinType = SkinTypeUpskin;
        part->skinParts.push_back(skinPart);

        skinPart.outline = downskin.difference(upskin);
        skinPart.skinType = SkinTypeDownskin;
        part->skinParts.push_back(skinPart);


        // for  now, there are only one upskin, downskin and sparse part (which each may contain many polygons.
        //Later, there may be also many bridge parts, each containing ecatly one polygon.
        double minAreaSize = (2 * M_PI * INT2MM(extrusionWidth) * INT2MM(extrusionWidth)) * 0.3;
        for(unsigned int partNr=0; partNr<part->skinParts.size();++partNr)
	    for(unsigned int polyNr=0; polyNr<part->skinParts[partNr].outline.size(); polyNr++)
	    {
			double area = INT2MM(INT2MM(fabs(part->skinParts[partNr].outline[polyNr].area())));
			if (area < minAreaSize) // Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill"
			{
		    	part->skinParts[partNr].outline.remove(polyNr);
		    	polyNr -= 1;
			}
	    }

        //part->sparseOutline = sparse.difference(part->skinOutline);
        skinPart.outline = sparse.difference(upskin.unionPolygons(downskin));
        skinPart.skinType = SkinTypeSparse;
        part->skinParts.push_back(skinPart);
    }
}



}//namespace cura
