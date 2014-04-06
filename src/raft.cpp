/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "raft.h"
#include "support.h"

void generateRaft(SliceDataStorage& storage, int distance)
{
    for(unsigned int volumeIdx = 0; volumeIdx < storage.volumes.size(); volumeIdx++)
    {
        if (storage.volumes[volumeIdx].layers.size() < 1) continue;
        SliceLayer* layer = &storage.volumes[volumeIdx].layers[0];
        for(unsigned int i=0; i<layer->parts.size(); i++)
        {
            storage.raftOutline = storage.raftOutline.unionPolygons(layer->parts[i].outline.offset(distance));
        }
    }

    SupportPolyGenerator supportGenerator(storage.support, 0);
    storage.raftOutline = storage.raftOutline.unionPolygons(supportGenerator.polygons.offset(distance));
    storage.raftOutline = storage.raftOutline.unionPolygons(storage.wipeTower.offset(distance));
}
