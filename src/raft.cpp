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

    SupportPolyGenerator supportGenerator(storage.support, 0, 0);
    storage.raftOutline = storage.raftOutline.unionPolygons(supportGenerator.polygons.offset(distance));
    storage.raftOutline = storage.raftOutline.unionPolygons(storage.wipeTower.offset(distance));
}

}//namespace cura
