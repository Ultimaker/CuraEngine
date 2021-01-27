//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningGenerator.h"
#include "LightningLayer.h"
#include "LightningTree.h"

#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SparsePointGridInclusive.h"

//____v

// TODO: distance field as sparse grid
// TODO: order pixels by dist to boundary

// TODO: junctions: create gap to avoid overextrusion at junctions

// TODO: avoid intersections between different branches

// TODO: improve tryGetNextPoint
//       Idea maybe something can be done with isolines for the distance function?
// TODO: Implement 'Truncate' ... is it needed to do it within the tree as well (see note-comment in function itself).
// TODO: The convert trees to lines 'algorithm' is way too simple right now (unless they're already going to be connected later).
// TODO: Merge LightningDistanceField into LightningLayer
// TODO: Lots of smaller TODO's in code itself, put on list!
// TODO: split radius into several parameters based on infill_line_distance, internal_overhang_angle, infill_overhang_angle, etc.

// TODO: improve connecting heuristic to favor connecting to shorter trees
// TODO: smoothing of junctions
// TODO: use sparse grid for efficiently looking up closest points in trees
// TODO: use swap with last trick when removing from unordered vector instead of moving all further elements back one place

// Implementation in Infill classes & elsewhere (not here):
// TODO: Outline offset, infill-overlap & perimeter gaps.
// TODO: Frontend, make sure it's enabled for 'actual' infill only (search for infill pattern).

// Stretch Goals:
// TODO: Check Unit Tests & Documentation.
// TODO: Result lines handled by Arachne insread :-)
// TODO: Also generate support instead of just infill (and enable that in frontend).
// TODO: Find a way to parallelize part(s) of it??
// TODO: G2/G3 to make trees properly curved? -> Should be in Arachne, not here, but still ;-)
// TODO: Generate double lined trees. -> mare stable and better printable without retractions or travels
// TODO: Generate all to-be-supported points at once instead of sequentially.
//       See branch interlocking_gen PolygonUtils::spreadDots
//       Or work with sparse grids.
// TODO: also straighten or smoothen junctions in trees

//____^

using namespace cura;

LightningGenerator::LightningGenerator(const coord_t& radius, const SliceMeshStorage& mesh) :
supporting_radius (radius)
{
    generateInitialInternalOverhangs(mesh, radius);
    generateTrees(mesh);  // NOTE: Ideally, these would not be in the constructor. TODO?: Rewrite 'Generator' as loose functions and perhaps a struct.
}

// Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
void LightningGenerator::generateInitialInternalOverhangs(const SliceMeshStorage& mesh, coord_t supporting_radius)
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

        Polygons overhang = infill_area_here.offset(-supporting_radius).difference(infill_area_above);

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
        std::unordered_set<std::shared_ptr<LightningTreeNode>> to_be_reconnected_tree_roots = current_lightning_layer.tree_roots;

        current_lightning_layer.generateNewTrees(overhang_per_layer[layer_id], current_outlines, supporting_radius);

        current_lightning_layer.reconnectRoots(to_be_reconnected_tree_roots, current_outlines, supporting_radius);

        // Initialize trees for next lower layer from the current one.
        if (layer_id == 0)
        {
            return;
        }
        const Polygons& below_outlines = infill_outlines[layer_id - 1];

        std::unordered_set<std::shared_ptr<LightningTreeNode>>& lower_trees = lightning_layers[layer_id - 1].tree_roots;
        for (auto& tree : current_lightning_layer.tree_roots)
        {
            tree->propagateToNextLayer
            (
                lower_trees,
                below_outlines,
                100, // TODO make pruning distance a separate parameter (ideally also as an anglem from which the tanget is used to compute the actual distance for a given layer)
                supporting_radius / 2 // TODO: should smooth-factor be a bit less tan the supporting radius?
            );
        }
    }
}
