//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningSupport.h"

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
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
    std::for_each(storage.meshes.begin(), storage.meshes.end(), [&](SliceMeshStorage& mesh) { generateSupportForMesh(mesh); });
    storage.support.layer_nr_max_filled_layer = lightning_layers.size() - 1;
    storage.support.lightning_supporter = this;
    storage.support.generated = true;
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
    const auto infill_wall_line_count = static_cast<coord_t>(mesh.settings.get<size_t>("support_wall_count"));
    const auto support_offset = mesh.settings.get<coord_t>("support_xy_distance");
    const auto infill_line_width = mesh.settings.get<coord_t>("support_line_width");
    const coord_t infill_wall_offset = -infill_wall_line_count * infill_line_width;

    infill_outlines = mesh.full_overhang_areas;
    no_root_areas.resize(infill_outlines.size());

    // For-each layer from top to bottom:
    for (int layer_id = mesh.full_overhang_areas.size() - 2; layer_id >= 0; layer_id--)
    {
        Polygons mesh_outlines;
        mesh.layers[layer_id].getOutlines(mesh_outlines);
        mesh_outlines = mesh_outlines.offset(std::max(support_offset, wall_supporting_radius));
        infill_outlines[layer_id] = infill_outlines[layer_id + 1].unionPolygons(mesh.full_overhang_areas[layer_id + 1]).difference(mesh_outlines);
        no_root_areas[layer_id] = mesh_outlines.offset(infill_line_width);
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

        // register all trees propagated from the previous layer as to-be-reconnected
        std::vector<LightningTreeNodeSPtr> to_be_reconnected_tree_roots = current_lightning_layer.tree_roots;

        current_lightning_layer.generateNewTrees(overhang_per_layer[layer_id], current_outlines, outlines_locator, supporting_radius, wall_supporting_radius);

        current_lightning_layer.reconnectRoots(to_be_reconnected_tree_roots, current_outlines, outlines_locator, supporting_radius, wall_supporting_radius);

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
            const coord_t start_prune_from = support_offset;  // TODO: senisble value
            tree->propagateToNextLayer(lower_trees, below_outlines, below_outlines_locator, prune_length, straightening_max_distance, locator_cell_size / 2, start_prune_from);
            //tree->visitBranches([&svg](const Point& a, const Point& b) { svg.writeLine(a, b); });
        }
    }
}

} // namespace cura
