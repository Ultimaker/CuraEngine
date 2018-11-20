//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <clipper.hpp>

#include "Application.h" //To get settings.
#include "raft.h"
#include "support.h"
#include "utils/math.h"

namespace cura {

void Raft::generate(SliceDataStorage& storage)
{
    assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const Settings& adhesion_extruder_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").settings;
    const coord_t distance = mesh_group_settings.get<coord_t>("raft_margin");
    constexpr bool include_support = true;
    constexpr bool include_prime_tower = true;
    storage.raftOutline = storage.getLayerOutlines(0, include_support, include_prime_tower).offset(distance, ClipperLib::jtRound);
    const coord_t shield_line_width_layer0 = adhesion_extruder_settings.get<coord_t>("skirt_brim_line_width");
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
    const coord_t smoothing = mesh_group_settings.get<coord_t>("raft_smoothing");
    storage.raftOutline = storage.raftOutline.offset(smoothing, ClipperLib::jtRound).offset(-smoothing, ClipperLib::jtRound); // remove small holes and smooth inward corners
}

coord_t Raft::getTotalThickness()
{
    const ExtruderTrain& raft_base_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_base_extruder_nr");
    const ExtruderTrain& raft_interface_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_interface_extruder_nr");
    const ExtruderTrain& raft_surface_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    return raft_base_extruder.settings.get<coord_t>("raft_base_thickness")
        + raft_interface_extruder.settings.get<coord_t>("raft_interface_thickness")
        + raft_surface_extruder.settings.get<size_t>("raft_surface_layers") * raft_surface_extruder.settings.get<coord_t>("raft_surface_thickness");
}

coord_t Raft::getZdiffBetweenRaftAndLayer1()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    const coord_t airgap = std::max(coord_t(0), mesh_group_settings.get<coord_t>("raft_airgap"));
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
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }

    const ExtruderTrain& raft_surface_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    return 2 + raft_surface_extruder.settings.get<size_t>("raft_surface_layers") + getFillerLayerCount();
}

bool Raft::isRaftBaseLayer(LayerIndex layer_nr)
{
    return layer_nr == LayerIndex(-static_cast<int>(getTotalExtraLayers()));
}

bool Raft::isRaftMiddleLayer(LayerIndex layer_nr)
{
    return layer_nr == LayerIndex(-static_cast<int>(getTotalExtraLayers()) + 1); // +1 raft interface layer
}

bool Raft::isRaftTopLayer(LayerIndex layer_nr)
{
    const ExtruderTrain& raft_surface_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    const int raft_surface_layers = static_cast<int>(raft_surface_extruder.settings.get<size_t>("raft_surface_layers"));
    const LayerIndex very_bottom_layer_nr = -static_cast<int>(getTotalExtraLayers());
    const LayerIndex first_raft_top_layer_nr = LayerIndex(very_bottom_layer_nr + 2);
    const LayerIndex last_raft_top_layer_nr = LayerIndex(very_bottom_layer_nr + 2 + raft_surface_layers - 1);
    return first_raft_top_layer_nr <= layer_nr && layer_nr <= last_raft_top_layer_nr;
}


}//namespace cura
