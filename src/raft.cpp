/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "raft.h"
#include "support.h"

namespace cura {

void generateRaft(SliceDataStorage& storage, int distance)
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.layers.size() < 1) continue;
        SliceLayer* layer = &mesh.layers[0];
        for(SliceLayerPart& part : layer->parts)
            storage.raftOutline = storage.raftOutline.unionPolygons(part.outline.offset(distance));
    }

    Polygons support;
    if (storage.support.generated) 
        support = storage.support.supportLayers[0].supportAreas;
    storage.raftOutline = storage.raftOutline.unionPolygons(support.offset(distance));
    storage.raftOutline = storage.raftOutline.unionPolygons(storage.wipeTower.offset(distance));
    storage.raftOutline = storage.raftOutline.unionPolygons(storage.draft_protection_screen.offset(distance));
}

}//namespace cura
