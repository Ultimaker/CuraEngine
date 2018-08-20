//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <clipper.hpp>

#include "Application.h" //To get settings.
#include "raft.h"
#include "support.h"
#include "utils/math.h"

namespace cura {

void Raft::generate(SliceDataStorage& storage, int distance)
{
    assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
    storage.raftOutline = storage.getLayerOutlines(0, true).offset(distance, ClipperLib::jtRound);
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    const coord_t shield_line_width_layer0 = train.settings.get<coord_t>("skirt_brim_line_width");
    if (storage.draft_protection_shield.size() > 0)
    {
        Polygons draft_shield_raft = storage.draft_protection_shield.offset(shield_line_width_layer0) // start half a line width outside shield
                                        .difference(storage.draft_protection_shield.offset(-distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftOutline = storage.raftOutline.unionPolygons(draft_shield_raft);
    }
    if (storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
    {
        const Polygons& ooze_shield = storage.oozeShield[0];
        Polygons ooze_shield_raft = ooze_shield.offset(shield_line_width_layer0) // start half a line width outside shield
                                        .difference(ooze_shield.offset(-distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftOutline = storage.raftOutline.unionPolygons(ooze_shield_raft);
    }
    const coord_t smoothing = train.settings.get<coord_t>("raft_smoothing");
    storage.raftOutline = storage.raftOutline.offset(smoothing, ClipperLib::jtRound).offset(-smoothing, ClipperLib::jtRound); // remove small holes and smooth inward corners
}

int Raft::getTotalThickness(const SliceDataStorage& storage)
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    return train.settings.get<coord_t>("raft_base_thickness")
        + train.settings.get<coord_t>("raft_interface_thickness")
        + train.settings.get<size_t>("raft_surface_layers") * train.settings.get<coord_t>("raft_surface_thickness");
}

int Raft::getZdiffBetweenRaftAndLayer1(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    const coord_t airgap = std::max(coord_t(0), train.settings.get<coord_t>("raft_airgap"));
    const coord_t layer_0_overlap = mesh_group_settings.get<coord_t>("layer_0_z_overlap");

    const coord_t layer_height_0 = mesh_group_settings.get<coord_t>("layer_height_0");

    const coord_t z_diff_raft_to_bottom_of_layer_1 = std::max(coord_t(0), airgap + layer_height_0 - layer_0_overlap);
    return z_diff_raft_to_bottom_of_layer_1;
}


int Raft::getFillerLayerCount(const SliceDataStorage& storage)
{
    const coord_t normal_layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
    const unsigned int filler_layer_count = round_divide(getZdiffBetweenRaftAndLayer1(storage), normal_layer_height);
    return filler_layer_count;
}

int Raft::getFillerLayerHeight(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        const coord_t normal_layer_height = mesh_group_settings.get<coord_t>("layer_height");
        return normal_layer_height;
    }
    const unsigned int filler_layer_height = round_divide(getZdiffBetweenRaftAndLayer1(storage), getFillerLayerCount(storage));
    return filler_layer_height;
}


int Raft::getTotalExtraLayers(const SliceDataStorage& storage)
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    if (train.settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    return 2 + train.settings.get<size_t>("raft_surface_layers") + getFillerLayerCount(storage);
}


}//namespace cura
