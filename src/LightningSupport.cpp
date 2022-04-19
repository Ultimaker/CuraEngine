//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningSupport.h"

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "TreeModelVolumes.h"
#include "infill/LightningTreeNode.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "settings/types/Angle.h"
#include "settings/types/Ratio.h"
#include "utils/algorithm.h"
#include "utils/IntPoint.h"
#include "utils/logoutput.h"
#include "utils/math.h"

//#include "utils/polygon.h"
//#include "utils/polygonUtils.h"

//#include <mutex>

//#include "utils/SVG.h"

namespace cura
{

LightningSupport::LightningSupport()
{
}

void LightningSupport::generateSupportAreas(SliceDataStorage& storage)
{
    const Settings& group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const bool global_use_tree_support =
        group_settings.get<bool>("support_enable") && group_settings.get<ESupportStructure>("support_structure") == ESupportStructure::LIGHTNING;
    if (! global_use_tree_support)
    {
        return;
    }
    const bool any_mesh_uses = std::any_of
        (
            storage.meshes.cbegin(),
            storage.meshes.cend(),
            [](const SliceMeshStorage& m)
            {
                return m.settings.get<bool>("support_enable") && m.settings.get<ESupportStructure>("support_structure") == ESupportStructure::LIGHTNING;
            }
        );
    if (! any_mesh_uses)
    {
        return;
    }

    // TODO: Only the last mesh gets properly processed, as each overwrites its predecessor at the moment.
    size_t max_layer = 0;
    std::for_each(storage.meshes.begin(), storage.meshes.end(), [&max_layer](SliceMeshStorage& mesh) { max_layer = std::max(max_layer, mesh.layers.size()); });
    generateTreeVolumes(storage, group_settings, max_layer);
    std::for_each(storage.meshes.begin(), storage.meshes.end(), [&](SliceMeshStorage& mesh) { generateSupportForMesh(mesh); });
    storage.support.layer_nr_max_filled_layer = lightning_layers.size() - 1;
    storage.support.lightning_supporter = this;
    storage.support.generated = true;
}

void LightningSupport::generateTreeVolumes(const SliceDataStorage& storage, const Settings& settings, const size_t max_layer)
{
    avoid_bp_extra_dist = 0;
    avoidance_envelope.resize(max_layer);

    const auto support_type = settings.get<ESupportType>("support_type");
    if (support_type == ESupportType::PLATFORM_ONLY)
    {
        const auto infill_extruder = settings.get<ExtruderTrain&>("support_extruder_nr");
        supporting_radius = std::max(settings.get<coord_t>("support_line_distance"), infill_extruder.settings.get<coord_t>("support_line_width")) / 2;
        const auto support_offset = settings.get<coord_t>("support_xy_distance");
        avoid_bp_extra_dist = support_offset / 2;

        TreeModelVolumes tree_model_volumes(storage, settings);
        
        Polygons prev_layer;
        for (size_t i_layer = 0; i_layer < max_layer; ++i_layer)
        {
            Polygons mesh_outlines;
            std::for_each(storage.meshes.begin(), storage.meshes.end(), [&](const SliceMeshStorage& mesh) { mesh.layers[i_layer].getOutlines(mesh_outlines); });

            avoidance_envelope[i_layer] =
                prev_layer
                .unionPolygons(mesh_outlines); // .offset(-supporting_radius);
                //.unionPolygons(tree_model_volumes.getCollision(avoid_bp_extra_dist, i_layer));
                //.unionPolygons(tree_model_volumes.getAvoidance(avoid_bp_extra_dist, i_layer));

            prev_layer = avoidance_envelope[i_layer].offset(-infill_extruder.settings.get<coord_t>("support_line_width"));
        }
    }
}

void LightningSupport::generateSupportForMesh(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::LIGHTNING)
    {
        return;
    }

    const auto infill_extruder = mesh.settings.get<ExtruderTrain&>("support_extruder_nr"); // TODO: This actually gets set a bit higher up in the chain for support.
    const auto layer_thickness = infill_extruder.settings.get<coord_t>("layer_height");  // Note: There's not going to be a layer below the first one, so the 'initial layer height' doesn't have to be taken into account.

    supporting_radius = std::max(infill_extruder.settings.get<coord_t>("support_line_distance"), infill_extruder.settings.get<coord_t>("support_line_width")) / 2;
    wall_supporting_radius = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_overhang_angle"));
    prune_length = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_prune_angle"));
    straightening_max_distance = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_straightening_angle"));
    // ^^^ TODO: separate values for lightning infill and lightning support ^^^

    generateInitialInternalOverhangs(mesh);
    generateTrees(mesh);
}

void LightningSupport::generateInitialInternalOverhangs(const SliceMeshStorage& mesh)
{
    overhang_per_layer = mesh.overhang_areas; //mesh.full_overhang_areas;
}

const LightningLayer& LightningSupport::getTreesForLayer(const size_t& layer_id) const
{
    assert(layer_id < lightning_layers.size());
    return lightning_layers[layer_id];
}

const Polygons& LightningSupport::getOutlinesForLayer(const size_t& layer_id) const
{
    assert(layer_id < lightning_layers.size());
    return infill_outlines[layer_id];
}

void LightningSupport::generateTrees(const SliceMeshStorage& mesh)
{
    lightning_layers.resize(mesh.full_overhang_areas.size());
    const auto support_offset = mesh.settings.get<coord_t>("support_xy_distance");
    const auto infill_line_width = mesh.settings.get<coord_t>("support_line_width");
    const auto support_type = mesh.settings.get<ESupportType>("support_type");
    const int attach_to_model_penalty = infill_line_width * 8;

    infill_outlines = mesh.full_overhang_areas;
    discourage_root_areas.resize(infill_outlines.size());

    //const coord_t avoid_bp_extra_dist = (support_type == ESupportType::PLATFORM_ONLY) ? support_offset / 2 : 0;

    // For-each layer from top to bottom:
    for (int layer_id = mesh.full_overhang_areas.size() - 2; layer_id >= 0; layer_id--)
    {
        Polygons mesh_outlines;
        mesh.layers[layer_id].getOutlines(mesh_outlines);
        mesh_outlines = mesh_outlines.offset(std::max(support_offset, wall_supporting_radius));
        //const auto& layer_volume = (support_type == ESupportType::PLATFORM_ONLY) ? tree_model_volumes->getAvoidance(avoid_bp_extra_dist, layer_id) : Polygons();
        //infill_outlines[layer_id] = infill_outlines[layer_id + 1].unionPolygons(mesh.full_overhang_areas[layer_id + 1]).difference(mesh_outlines.unionPolygons(layer_volume));
        infill_outlines[layer_id] = infill_outlines[layer_id + 1].unionPolygons(mesh.full_overhang_areas[layer_id + 1]).difference(mesh_outlines);
        discourage_root_areas[layer_id] = mesh_outlines.offset(infill_line_width);
    }
    if (avoid_bp_extra_dist > 0)
    {
        for (int layer_id = mesh.full_overhang_areas.size() - 2; layer_id >= 0; layer_id--)
        {
            infill_outlines[layer_id] = infill_outlines[layer_id].offset(avoid_bp_extra_dist);
        }
    }

    // For various operations its beneficial to quickly locate nearby features on the polygon:
    const size_t top_layer_id = mesh.full_overhang_areas.size() - 1;
    auto outlines_locator_ptr = PolygonUtils::createLocToLineGrid(infill_outlines[top_layer_id], locator_cell_size);

    // For-each layer from top to bottom:
    for (int layer_id = top_layer_id; layer_id >= 0; layer_id--)
    {
        LightningLayer& current_lightning_layer = lightning_layers[layer_id];
        Polygons& current_outlines = infill_outlines[layer_id];
        const auto& outlines_locator = *outlines_locator_ptr;

        const std::function<int(Point)>& root_location_penalty_function
        (
            [&](Point p)
            {
                return discourage_root_areas[layer_id].inside(p) ? attach_to_model_penalty : 0;
            }
        );

        // register all trees propagated from the previous layer as to-be-reconnected
        std::vector<LightningTreeNodeSPtr> to_be_reconnected_tree_roots = current_lightning_layer.tree_roots;

        current_lightning_layer.generateNewTrees(overhang_per_layer[layer_id], current_outlines, outlines_locator, supporting_radius, wall_supporting_radius, root_location_penalty_function);
        current_lightning_layer.reconnectRoots(to_be_reconnected_tree_roots, current_outlines, outlines_locator, supporting_radius, wall_supporting_radius, root_location_penalty_function);
        if (!avoidance_envelope.empty())
        {
            for (auto tree : current_lightning_layer.tree_roots)
            {
                tree->moveOutsideOf(avoidance_envelope[layer_id], supporting_radius * supporting_radius * 4);
            }
        }

        // Initialize trees for next lower layer from the current one.
        if (layer_id == 0)
        {
            return;
        }
        const Polygons& below_outlines = infill_outlines[layer_id - 1];
        outlines_locator_ptr = PolygonUtils::createLocToLineGrid(below_outlines, locator_cell_size);
        const auto& below_outlines_locator = *outlines_locator_ptr;

        std::vector<LightningTreeNodeSPtr>& lower_trees = lightning_layers[layer_id - 1].tree_roots;
        for (auto& tree : current_lightning_layer.tree_roots)
        {
            const coord_t start_prune_from = supporting_radius * 8;
            //const Polygons outside_poly = avoid_bp_extra_dist > 0 ? tree_model_volumes->getAvoidance(avoid_bp_extra_dist, layer_id) : Polygons();
            tree->propagateToNextLayer
            (
                lower_trees,
                below_outlines,
                below_outlines_locator,
                prune_length,
                straightening_max_distance,
                locator_cell_size / 2,
                start_prune_from
            );
            if (! avoidance_envelope.empty())
            {
                tree->moveOutsideOf(avoidance_envelope[layer_id], supporting_radius * supporting_radius * 4);
            }
            //tree->visitBranches([&svg](const Point& a, const Point& b) { svg.writeLine(a, b); });
        }
    }
}

} // namespace cura
