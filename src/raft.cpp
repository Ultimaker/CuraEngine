// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "raft.h"

#include <polyclipping/clipper.hpp>

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/EnumSettings.h" //For EPlatformAdhesion.
#include "sliceDataStorage.h"
#include "utils/math.h"

namespace cura
{

void Raft::generate(SliceDataStorage& storage)
{
    assert(
        storage.raftBaseOutline.size() == 0 && storage.raftInterfaceOutline.size() == 0 && storage.raftSurfaceOutline.size() == 0
        && "Raft polygon isn't generated yet, so should be empty!");
    const Settings& settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<ExtruderTrain&>("raft_base_extruder_nr").settings_;
    constexpr bool include_support = true;
    constexpr bool dont_include_prime_tower = false; // Prime tower raft will be handled separately in 'storage.primeRaftOutline'; see below.
    const auto raft_base_margin = settings.get<coord_t>("raft_base_margin");
    const auto raft_interface_margin = settings.get<coord_t>("raft_interface_margin");
    const auto raft_surface_margin = settings.get<coord_t>("raft_surface_margin");

    storage.raftBaseOutline = storage.raftSurfaceOutline = storage.raftInterfaceOutline = storage.getLayerOutlines(0, include_support, dont_include_prime_tower);
    storage.raftBaseOutline = storage.raftBaseOutline.offset(raft_base_margin, ClipperLib::jtRound);
    storage.raftInterfaceOutline = storage.raftInterfaceOutline.offset(raft_interface_margin, ClipperLib::jtRound);
    storage.raftSurfaceOutline = storage.raftSurfaceOutline.offset(raft_surface_margin, ClipperLib::jtRound);

    const coord_t shield_line_width_layer0 = settings.get<coord_t>("skirt_brim_line_width");
    const coord_t max_raft_distance = std::max(std::max(raft_base_margin, raft_interface_margin), raft_surface_margin);
    if (storage.draft_protection_shield.size() > 0)
    {
        Polygons draft_shield_raft
            = storage.draft_protection_shield
                  .offset(shield_line_width_layer0) // start half a line width outside shield
                  .difference(storage.draft_protection_shield.offset(-max_raft_distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftBaseOutline = storage.raftBaseOutline.unionPolygons(draft_shield_raft);
        storage.raftSurfaceOutline = storage.raftSurfaceOutline.unionPolygons(draft_shield_raft);
        storage.raftInterfaceOutline = storage.raftInterfaceOutline.unionPolygons(draft_shield_raft);
    }
    if (storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
    {
        const Polygons& ooze_shield = storage.oozeShield[0];
        Polygons ooze_shield_raft = ooze_shield
                                        .offset(shield_line_width_layer0) // start half a line width outside shield
                                        .difference(ooze_shield.offset(-max_raft_distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
        storage.raftBaseOutline = storage.raftBaseOutline.unionPolygons(ooze_shield_raft);
        storage.raftSurfaceOutline = storage.raftSurfaceOutline.unionPolygons(ooze_shield_raft);
        storage.raftInterfaceOutline = storage.raftInterfaceOutline.unionPolygons(ooze_shield_raft);
    }

    const auto remove_inside_corners = [](Polygons& outline, bool remove_inside_corners, coord_t smoothing)
    {
        if (remove_inside_corners)
        {
            // Make all parts convex. We could take the convex hull of the union of all parts, but that would
            // result in a massive brim if you would have models in all corners of the build plate. We instead
            // perform a convex hull operation on each polygon part separately, this will result in much smaller
            // raft islands. However, this might result in inside corners in the raft outline in the situation
            // where after the merge operations some parts become connected. To properly make sure that there are
            // no inside corners we perform a convex hull operation followed by a merge. If the number of polygon
            // parts no longer decrease we have found the polygons that have both no longer inside corners and
            // occupy the smallest possible area.
            outline = outline.unionPolygons();
            auto outline_parts = outline.unionPolygons().splitIntoParts();
            auto nr_of_parts = outline_parts.size();

            while (true)
            {
                outline.clear();

                for (auto& part : outline_parts)
                {
                    part.makeConvex();
                    outline.add(part);
                }

                outline = outline.unionPolygons();
                outline_parts = outline.splitIntoParts();
                const auto new_nr_of_parts = outline_parts.size();

                if (new_nr_of_parts > nr_of_parts)
                {
                    // from the recursive convex hull + merge operation the number of parts cannot logically increase
                    // if it does increase, and allow the loop to continue we might get into an infinite loop; so break out of the loop
                    // this might produce a raft with inside corners, but that is better than an infinite loop
                    spdlog::warn("Error while removing inside corners from raft; merge operation increased the number of parts");
                    assert(false);
                    break;
                }

                if (new_nr_of_parts == nr_of_parts)
                {
                    break;
                }
                nr_of_parts = new_nr_of_parts;
            }
        }
        else
        {
            outline = outline.offset(smoothing, ClipperLib::jtRound).offset(-smoothing, ClipperLib::jtRound);
        }
    };
    remove_inside_corners(storage.raftBaseOutline, settings.get<bool>("raft_base_remove_inside_corners"), settings.get<coord_t>("raft_base_smoothing"));
    remove_inside_corners(storage.raftInterfaceOutline, settings.get<bool>("raft_interface_remove_inside_corners"), settings.get<coord_t>("raft_interface_smoothing"));
    remove_inside_corners(storage.raftSurfaceOutline, settings.get<bool>("raft_surface_remove_inside_corners"), settings.get<coord_t>("raft_surface_smoothing"));

    if (storage.primeTower.enabled_ && ! storage.primeTower.would_have_actual_tower_)
    {
        // Find out if the prime-tower part of the raft still needs to be printed, even if there is no actual tower.
        // This will only happen if the different raft layers are printed by different extruders.
        const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
        const size_t base_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr_;
        const size_t interface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr_;
        const size_t surface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr_;
        if (base_extruder_nr == interface_extruder_nr && base_extruder_nr == surface_extruder_nr)
        {
            return;
        }
    }
}

coord_t Raft::getTotalThickness()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ExtruderTrain& base_train = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr");
    const ExtruderTrain& interface_train = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr");
    const ExtruderTrain& surface_train = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    return base_train.settings_.get<coord_t>("raft_base_thickness")
         + interface_train.settings_.get<size_t>("raft_interface_layers") * interface_train.settings_.get<coord_t>("raft_interface_thickness")
         + surface_train.settings_.get<size_t>("raft_surface_layers") * surface_train.settings_.get<coord_t>("raft_surface_thickness");
}

coord_t Raft::getZdiffBetweenRaftAndLayer0()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    const coord_t airgap = std::max(coord_t(0), train.settings_.get<coord_t>("raft_airgap"));
    return airgap;
}

size_t Raft::getFillerLayerCount()
{
    const coord_t normal_layer_height = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<coord_t>("layer_height");
    return round_divide(getZdiffBetweenRaftAndLayer0(), normal_layer_height);
}

coord_t Raft::getFillerLayerHeight()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        const coord_t normal_layer_height = mesh_group_settings.get<coord_t>("layer_height");
        return normal_layer_height;
    }

    return round_divide(getZdiffBetweenRaftAndLayer0(), getFillerLayerCount());
}


size_t Raft::getTotalExtraLayers()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ExtruderTrain& base_train = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr");
    const ExtruderTrain& interface_train = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr");
    const ExtruderTrain& surface_train = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    if (base_train.settings_.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        return 0;
    }
    return 1 + interface_train.settings_.get<size_t>("raft_interface_layers") + surface_train.settings_.get<size_t>("raft_surface_layers") + getFillerLayerCount();
}

Raft::LayerType Raft::getLayerType(LayerIndex layer_index)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const ExtruderTrain& base_train = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr");
    const ExtruderTrain& interface_train = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr");
    const ExtruderTrain& surface_train = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
    const auto airgap = Raft::getFillerLayerCount();
    const auto interface_layers = interface_train.settings_.get<size_t>("raft_interface_layers");
    const auto surface_layers = surface_train.settings_.get<size_t>("raft_surface_layers");

    if (layer_index < -airgap - surface_layers - interface_layers)
    {
        return LayerType::RaftBase;
    }
    if (layer_index < -airgap - surface_layers)
    {
        return LayerType::RaftInterface;
    }
    if (layer_index < -airgap)
    {
        return LayerType::RaftSurface;
    }
    else if (layer_index < 0)
    {
        return LayerType::Airgap;
    }
    else
    {
        return LayerType::Model;
    }
}


} // namespace cura
