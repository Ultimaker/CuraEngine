/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_H
#define SKIRT_H

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength)
{
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        for(unsigned int volumeIdx = 0; volumeIdx < storage.volumes.size(); volumeIdx++)
        {
            if (storage.volumes[volumeIdx].layers.size() < 1) continue;
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[0];
            for(unsigned int i=0; i<layer->parts.size(); i++)
            {
                storage.skirt = storage.skirt.unionPolygons(layer->parts[i].outline.offset(distance + extrusionWidth * skirtNr + extrusionWidth / 2));
            }
        }
        
        int lenght = storage.skirt.polygonLength();
        if (skirtNr + 1 >= count && lenght > 0 && lenght < minLength)
            count++;
    }
}

#endif//SKIRT_H
