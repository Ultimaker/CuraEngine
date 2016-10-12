/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <clipper/clipper.hpp>

#include "utils/math.h"
#include "raft.h"
#include "support.h"

namespace cura {

void Raft::generate(SliceDataStorage& storage, int distance)
{
    assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
    storage.raftOutline = storage.getLayerOutlines(0, true).offset(distance, ClipperLib::jtRound);
    const int shield_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"))->getSettingInMicrons("skirt_brim_line_width");
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

int Raft::getTotalThickness(const SliceDataStorage& storage)
{
    const ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"));
    return train.getSettingInMicrons("raft_base_thickness")
        + train.getSettingInMicrons("raft_interface_thickness")
        + train.getSettingAsCount("raft_surface_layers") * train.getSettingInMicrons("raft_surface_thickness");
}

int Raft::getZdiffBetweenRaftAndLayer1(const SliceDataStorage& storage)
{
    const ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"));
    if (storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    const int64_t airgap = std::max(0, train.getSettingInMicrons("raft_airgap"));
    const int64_t layer_0_overlap = storage.getSettingInMicrons("layer_0_z_overlap");

    const int64_t layer_height_0 = storage.getSettingInMicrons("layer_height_0");

    const int64_t z_diff_raft_to_bottom_of_layer_1 = std::max(int64_t(0), airgap + layer_height_0 - layer_0_overlap);
    return z_diff_raft_to_bottom_of_layer_1;
}


int Raft::getFillerLayerCount(const SliceDataStorage& storage)
{
    const int64_t normal_layer_height = storage.getSettingInMicrons("layer_height");
    const unsigned int filler_layer_count = round_divide(getZdiffBetweenRaftAndLayer1(storage), normal_layer_height);
    return filler_layer_count;
}

int Raft::getFillerLayerHeight(const SliceDataStorage& storage)
{
    if (storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        const int64_t normal_layer_height = storage.getSettingInMicrons("layer_height");
        return normal_layer_height;
    }
    const unsigned int filler_layer_height = round_divide(getZdiffBetweenRaftAndLayer1(storage), getFillerLayerCount(storage));
    return filler_layer_height;
}


int Raft::getTotalExtraLayers(const SliceDataStorage& storage)
{
    const ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("adhesion_extruder_nr"));
    return 2 + train.getSettingAsCount("raft_surface_layers") + getFillerLayerCount(storage);
}


}//namespace cura
