//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <polyclipping/clipper.hpp>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "raft.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "support.h"
#include "settings/EnumSettings.h" //For EPlatformAdhesion.
#include "utils/math.h"

namespace cura
{

void Raft::generate(SliceDataStorage& storage)
{
    assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
    const Settings& settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").settings;
    const coord_t distance = settings.get<coord_t>("raft_margin");
    constexpr bool include_support = true;
    constexpr bool include_prime_tower = true;
    storage.raftOutline = storage.getLayerOutlines(0, include_support, include_prime_tower).offset(distance, ClipperLib::jtRound);
    const coord_t shield_line_width_layer0 = settings.get<coord_t>("skirt_brim_line_width");
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
    const coord_t smoothing = settings.get<coord_t>("raft_smoothing");
    storage.raftOutline = storage.raftOutline.offset(smoothing, ClipperLib::jtRound).offset(-smoothing, ClipperLib::jtRound); // remove small holes and smooth inward corners
}

coord_t Raft::getTotalThickness()
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    return train.settings.get<coord_t>("raft_base_thickness")
        + train.settings.get<coord_t>("raft_interface_thickness")
        + train.settings.get<size_t>("raft_surface_layers") * train.settings.get<coord_t>("raft_surface_thickness");
}

coord_t Raft::getZdiffBetweenRaftAndLayer1()
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

size_t Raft::getFillerLayerCount()
{
    const coord_t normal_layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
    return round_divide(getZdiffBetweenRaftAndLayer1(), normal_layer_height);
}

coord_t Raft::getFillerLayerHeight()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        const coord_t normal_layer_height = mesh_group_settings.get<coord_t>("layer_height");
        return normal_layer_height;
    }
    return round_divide(getZdiffBetweenRaftAndLayer1(), getFillerLayerCount());
}


size_t Raft::getTotalExtraLayers()
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    if (train.settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    return 2 + train.settings.get<size_t>("raft_surface_layers") + getFillerLayerCount();
}


}//namespace cura
