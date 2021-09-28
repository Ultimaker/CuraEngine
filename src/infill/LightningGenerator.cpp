//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningGenerator.h"
#include "LightningLayer.h"
#include "LightningTreeNode.h"

#include "../ExtruderTrain.h"
#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SparsePointGridInclusive.h"

//____v

// TODO: more sophisticated heuristics for making nice trees
// TODO: improve connecting heuristic to favor connecting to shorter trees

// TODO: avoid intersections between different branches

// TODO: reconnectRoots : change which node of a tree is the root when that would be better

// TODO: hardcoded value in LightningTreeNode::straighten : small_branch

// Implementation in Infill classes & elsewhere (not here):
// TODO: Outline offset, infill-overlap & perimeter gaps.
// TODO: Frontend, make sure it's enabled for 'actual' infill only (search for infill pattern).

// Stretch Goals:
// TODO: Check Unit Tests & Documentation.
// TODO: optimization: let the square grid store the closest point on boundary
// TODO: optimization: only compute the closest dist to / point on boundary for the outer cells and flood-fill the rest
// TODO: Result lines handled by Arachne insread :-)
// TODO: Also generate support instead of just infill (and enable that in frontend).
// TODO: Find a way to parallelize part(s) of it??
// TODO: G2/G3 to make trees properly curved? -> Should be in Arachne, not here, but still ;-)
// TODO: Generate double lined trees. -> mare stable and better printable without retractions or travels
// TODO: Generate all to-be-supported points at once instead of sequentially.
//       See branch interlocking_gen PolygonUtils::spreadDots
//       Or work with sparse grids.



// TODO s from in the code:

// make distance field cell size configurable? Whats the best value?
// LightningLayer::getBestGroundingLocation : make boundary size in which we ignore the valence rule configurable
// LightningLayer::convertToLines : allow for polylines, i.e. merge Tims PR about polyline fixes
//
//

// Small code style etc:
// TODO: Merge LightningDistanceField into LightningLayer
// TODO: use swap with last trick when removing from unordered vector instead of moving all further elements back one place
// LightningLayer::generateNewTrees : remove debug_max_iterations ?
// move LightningDistanceField to its own file
// move complexer computations outside of LightningGenerator constructur

//____^

using namespace cura;

LightningGenerator::LightningGenerator(const SliceMeshStorage& mesh)
{
    const auto infill_extruder = mesh.settings.get<ExtruderTrain&>("infill_extruder_nr");
    const coord_t layer_thickness = infill_extruder.settings.get<coord_t>("layer_height");  // Note: Currently no initial_layer, probably not necesary, since infill doesn't usually start on layer 0

    supporting_radius = infill_extruder.settings.get<coord_t>("infill_line_distance") / 2;
    wall_supporting_radius = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_overhang_angle"));
    prune_length = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_prune_angle"));
    straightening_max_distance = layer_thickness * std::tan(infill_extruder.settings.get<AngleRadians>("lightning_infill_straightening_angle"));

    generateInitialInternalOverhangs(mesh);
    generateTrees(mesh);  // NOTE: Ideally, these would not be in the constructor. TODO?: Rewrite 'Generator' as loose functions and perhaps a struct.
}

// Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
void LightningGenerator::generateInitialInternalOverhangs(const SliceMeshStorage& mesh)
{
    overhang_per_layer.resize(mesh.layers.size());

    Polygons infill_area_above;
    for (int layer_nr = mesh.layers.size() - 1; layer_nr >= 0; layer_nr--)
    {
        const SliceLayer& current_layer = mesh.layers[layer_nr];
        Polygons infill_area_here;
        for (auto& part : current_layer.parts)
        {
            infill_area_here.add(part.getOwnInfillArea());
        }

        Polygons overhang = infill_area_here.offset(-wall_supporting_radius).difference(infill_area_above);

        overhang_per_layer[layer_nr] = overhang;
        infill_area_above = std::move(infill_area_here);
    }
}

const LightningLayer& LightningGenerator::getTreesForLayer(const size_t& layer_id)
{
    assert(layer_id < lightning_layers.size());
    return lightning_layers[layer_id];
}

void LightningGenerator::generateTrees(const SliceMeshStorage& mesh)
{
    lightning_layers.resize(mesh.layers.size());

    std::vector<Polygons> infill_outlines;
    infill_outlines.insert(infill_outlines.end(), mesh.layers.size(), Polygons());

    // For-each layer from top to bottom:
    for (int layer_id = mesh.layers.size() - 1; layer_id >= 0; layer_id--)
    {
        for (const auto& part : mesh.layers[layer_id].parts)
        {
            infill_outlines[layer_id].add(part.getOwnInfillArea());
        }
    }

    // For-each layer from top to bottom:
    for (int layer_id = mesh.layers.size() - 1; layer_id >= 0; layer_id--)
    {
        LightningLayer& current_lightning_layer = lightning_layers[layer_id];
        Polygons& current_outlines = infill_outlines[layer_id];

        // register all trees propagated from the previous layer as to-be-reconnected
        std::vector<std::shared_ptr<LightningTreeNode>> to_be_reconnected_tree_roots = current_lightning_layer.tree_roots;

        current_lightning_layer.generateNewTrees(overhang_per_layer[layer_id], current_outlines, supporting_radius);

        current_lightning_layer.reconnectRoots(to_be_reconnected_tree_roots, current_outlines, supporting_radius, wall_supporting_radius);

        // Initialize trees for next lower layer from the current one.
        if (layer_id == 0)
        {
            return;
        }
        const Polygons& below_outlines = infill_outlines[layer_id - 1];

        std::vector<std::shared_ptr<LightningTreeNode>>& lower_trees = lightning_layers[layer_id - 1].tree_roots;
        for (auto& tree : current_lightning_layer.tree_roots)
        {
            tree->propagateToNextLayer(lower_trees, below_outlines, prune_length, straightening_max_distance);
        }
    }
}
