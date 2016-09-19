/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <clipper/clipper.hpp>

#include "raft.h"
#include "support.h"

namespace cura {

void Raft::generate(SliceDataStorage& storage, int distance)
{
    assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
    storage.raftOutline = storage.getLayerOutlines(0, true).offset(distance, ClipperLib::jtRound);
    int shield_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"))->getSettingInMicrons("skirt_brim_line_width");
    if (storage.draft_protection_shield.size() > 0)
    {
        Polygons draft_shield_raft = storage.draft_protection_shield.offset(shield_line_width) // start half a line width outside shield
                                        .difference(storage.draft_protection_shield.offset(-distance - shield_line_width / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftOutline = storage.raftOutline.unionPolygons(draft_shield_raft);
    }
    if (storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
    {
        const Polygons& ooze_shield = storage.oozeShield[0];
        Polygons ooze_shield_raft = ooze_shield.offset(shield_line_width) // start half a line width outside shield
                                        .difference(ooze_shield.offset(-distance - shield_line_width / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftOutline = storage.raftOutline.unionPolygons(ooze_shield_raft);
    }
    storage.raftOutline = storage.raftOutline.offset(1000).offset(-1000); // remove small holes
}

}//namespace cura
