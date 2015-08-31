/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "raft.h"
#include "support.h"

namespace cura {

void generateRaft(SliceDataStorage& storage, int distance)
{
    if (storage.draft_protection_shield.size() > 0)
    {
        storage.raftOutline = storage.raftOutline.unionPolygons(storage.draft_protection_shield.offset(distance));
    }
    else if (storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
    {
        storage.raftOutline = storage.raftOutline.unionPolygons(storage.oozeShield[0].offset(distance));
    }
    else 
    {
        storage.raftOutline = storage.getLayerOutlines(0, true);
    }
}

}//namespace cura
