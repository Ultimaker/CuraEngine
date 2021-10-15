//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningGenerator.h"
#include "LightningLayer.h"
#include "LightningTreeNode.h"

#include "../ExtruderTrain.h"
#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SparsePointGridInclusive.h"

/* Possible future tasks/optimizations,etc.:
 * - Improve connecting heuristic to favor connecting to shorter trees
 * - Change which node of a tree is the root when that would be better in reconnectRoots.
 * - (For implementation in Infill classes & elsewhere): Outline offset, infill-overlap & perimeter gaps.
 * - Allow for polylines, i.e. merge Tims PR about polyline fixes
 * - Unit Tests?
 * - Optimization: let the square grid store the closest point on boundary
 * - Optimization: only compute the closest dist to / point on boundary for the outer cells and flood-fill the rest
 * - Make a pass with Arachne over the output. Somehow.
 * - Generate all to-be-supported points at once instead of sequentially: See branch interlocking_gen PolygonUtils::spreadDots (Or work with sparse grids.)
 * - Lots of magic values ... to many to parameterize. But are they the best?
 * - Move more complex computations from LightningGenerator constructor to elsewhere.
 */

using namespace cura;

LightningGenerator::LightningGenerator(const SliceMeshStorage& mesh)
{
    const auto infill_extruder = mesh.settings.get<ExtruderTrain&>("infill_extruder_nr");
    const auto layer_thickness = infill_extruder.settings.get<coord_t>("layer_height");  // Note: There's not going to be a layer below the first one, so the 'initial layer height' doesn't have to be taken into account.

    supporting_radius = std::max(infill_extruder.settings.get<coord_t>("infill_line_distance"), infill_extruder.settings.get<coord_t>("infill_line_width")) / 2;
    wall_supporting_radius = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_overhang_angle"));
    prune_length = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_prune_angle"));
    straightening_max_distance = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_straightening_angle"));

    generateInitialInternalOverhangs(mesh);
    generateTrees(mesh);
}

void LightningGenerator::generateInitialInternalOverhangs(const SliceMeshStorage& mesh)
{
    overhang_per_layer.resize(mesh.layers.size());
    const auto infill_wall_line_count = static_cast<coord_t>(mesh.settings.get<size_t>("infill_wall_line_count"));
    const auto infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
    const coord_t infill_wall_offset = - infill_wall_line_count *  infill_line_width;

    Polygons infill_area_above;
    //Iterate from top to bottom, to subtract the overhang areas above from the overhang areas on the layer below, to get only overhang in the top layer where it is overhanging.
    for (int layer_nr = mesh.layers.size() - 1; layer_nr >= 0; layer_nr--)
    {
        const SliceLayer& current_layer = mesh.layers[layer_nr];
        Polygons infill_area_here;
        for (auto& part : current_layer.parts)
        {
            infill_area_here.add(part.getOwnInfillArea().offset(infill_wall_offset));
        }

        //Remove the part of the infill area that is already supported by the walls.
        Polygons overhang = infill_area_here.offset(-wall_supporting_radius).difference(infill_area_above);

        overhang_per_layer[layer_nr] = overhang;
        infill_area_above = std::move(infill_area_here);
    }
}

const LightningLayer& LightningGenerator::getTreesForLayer(const size_t& layer_id) const
{
    assert(layer_id < lightning_layers.size());
    return lightning_layers[layer_id];
}

void LightningGenerator::generateTrees(const SliceMeshStorage& mesh)
{
    lightning_layers.resize(mesh.layers.size());
    const auto infill_wall_line_count = static_cast<coord_t>(mesh.settings.get<size_t>("infill_wall_line_count"));
    const auto infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
    const coord_t infill_wall_offset = - infill_wall_line_count *  infill_line_width;

    std::vector<Polygons> infill_outlines;
    infill_outlines.insert(infill_outlines.end(), mesh.layers.size(), Polygons());

    // For-each layer from top to bottom:
    for (int layer_id = mesh.layers.size() - 1; layer_id >= 0; layer_id--)
    {
        for (const auto& part : mesh.layers[layer_id].parts)
        {
            infill_outlines[layer_id].add(part.getOwnInfillArea().offset(infill_wall_offset));
        }
    }

    // For various operations its beneficial to quickly locate nearby features on the polygon:
    const size_t top_layer_id = mesh.layers.size() - 1;
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
            tree->propagateToNextLayer(lower_trees, below_outlines, below_outlines_locator, prune_length, straightening_max_distance);
        }
    }
}
