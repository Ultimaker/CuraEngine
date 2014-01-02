/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skirt.h"
#include "support.h"

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength, int initialLayerHeight)
{
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        int offsetDistance = distance + extrusionWidth * skirtNr + extrusionWidth / 2;
        
        Polygons skirtPolygons(storage.wipeTower.offset(offsetDistance));
        for(unsigned int volumeIdx = 0; volumeIdx < storage.volumes.size(); volumeIdx++)
        {
            if (storage.volumes[volumeIdx].layers.size() < 1) continue;
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[0];
            for(unsigned int i=0; i<layer->parts.size(); i++)
            {
                skirtPolygons = skirtPolygons.unionPolygons(layer->parts[i].outline.offset(offsetDistance));
            }
        }
        
        SupportPolyGenerator supportGenerator(storage.support, initialLayerHeight);
        skirtPolygons = skirtPolygons.unionPolygons(supportGenerator.polygons.offset(offsetDistance));

        storage.skirt.add(skirtPolygons);
        
        int lenght = storage.skirt.polygonLength();
        if (skirtNr + 1 >= count && lenght > 0 && lenght < minLength)
            count++;
    }
}
