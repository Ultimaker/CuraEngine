//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningGenerator.h"
#include "LightningTree.h"

#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SparsePointGridInclusive.h"

//____v

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

coord_t LightningLayer::getWeightedDistance(const Point& boundary_loc, const Point& unsupported_loc)
{
    return vSize(boundary_loc - unsupported_loc);
}

LightningDistanceField::LightningDistanceField
(
    const coord_t& radius,
    const Polygons& current_outline,
    const Polygons& current_overhang,
    const std::unordered_set<std::shared_ptr<LightningTreeNode>>& initial_trees
)
{
    supporting_radius = radius;
    Polygons supporting_polylines = current_outline;
    for (PolygonRef poly : supporting_polylines)
    {
        if ( ! poly.empty())
        {
            poly.add(poly[0]); // add start so that the polyline is closed
        }
    }

    const LightningTreeNode::branch_visitor_func_t add_offset_branch_func =
        [&](const Point& parent, const Point& child)
        {
            supporting_polylines.addLine(parent, child);
        };
    for (const auto& tree : initial_trees)
    {
        tree->visitBranches(add_offset_branch_func);
    }
    supported = supporting_polylines.offsetPolyLine(supporting_radius);
    unsupported = current_overhang.difference(supported);
}

bool LightningDistanceField::tryGetNextPoint(Point* p, coord_t supporting_radius) const
{
    if (unsupported.area() < 25)
    {
        return false;
    }
    coord_t total_length = unsupported[0].polygonLength();
    coord_t dist_to_point_on_boundary = std::rand() % total_length;
    ClosestPolygonPoint cpp = PolygonUtils::walk(ClosestPolygonPoint(unsupported[0][0], 0, unsupported[0]), dist_to_point_on_boundary);
    *p = PolygonUtils::moveInside(cpp, supporting_radius);
    if ( ! unsupported.inside(*p))
    {
        PolygonUtils::moveInside(unsupported, *p, supporting_radius / 2);
    }
    // NOTE: it's okay for the rare case where a point ends up outside; it's just an inefficient tree branch.
    return true;
}

void LightningDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    Polygons line;
    line.addLine(to_node, added_leaf);
    Polygons offsetted = line.offsetPolyLine(supporting_radius, ClipperLib::jtRound);
    supported = supported.unionPolygons(offsetted);
    unsupported = unsupported.difference(supported);
}

// -- -- -- -- -- --
// -- -- -- -- -- --

Point GroundingLocation::p() const
{
    if (tree_node != nullptr)
    {
        return tree_node->getLocation();
    }
    else
    {
        assert(boundary_location);
        return boundary_location->p();
    }
}

LightningTreeNode::node_visitor_func_t getAddToLocatorFunc(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator)
{
    return
        [&tree_node_locator](std::shared_ptr<LightningTreeNode> node)
        {
            tree_node_locator.insert(node->getLocation(), node);
        };
}

void LightningLayer::fillLocator(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator, const std::unordered_set<std::shared_ptr<LightningTreeNode>>& excluded_trees_by_root)
{
    const LightningTreeNode::node_visitor_func_t add_node_to_locator_func = getAddToLocatorFunc(tree_node_locator);
    for (auto& tree : tree_roots)
    {
        if (excluded_trees_by_root.count(tree) > 0)
        {
            continue;
        }
        tree->visitNodes(add_node_to_locator_func);
    }
}

void LightningLayer::generateNewTrees(const Polygons& current_overhang, Polygons& current_outlines, coord_t supporting_radius)
{
    LightningDistanceField distance_field(supporting_radius, current_outlines, current_overhang, tree_roots);

    constexpr coord_t locator_cell_size = 2000;
    SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>> tree_node_locator(locator_cell_size);
    fillLocator(tree_node_locator);

    constexpr size_t debug_max_iterations = 9999; // TODO: remove
    size_t i_debug = 0;

    // Until no more points need to be added to support all:
    // Determine next point from tree/outline areas via distance-field
    Point unsupported_location;
    while (distance_field.tryGetNextPoint(&unsupported_location, supporting_radius)    && i_debug < debug_max_iterations)
    {
        ++i_debug;

        GroundingLocation grounding_loc = getBestGroundingLocation(unsupported_location, current_outlines, supporting_radius, tree_node_locator);

        // TODO: update unsupported_location to lie closer to grounding_loc

        attach(unsupported_location, grounding_loc);

        // update distance field
        distance_field.update(grounding_loc.p(), unsupported_location);
    }
}

GroundingLocation LightningLayer::getBestGroundingLocation(const Point& unsupported_location, const Polygons& current_outlines, const coord_t supporting_radius, const SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator, const std::shared_ptr<LightningTreeNode>& exclude_tree)
{
    ClosestPolygonPoint cpp = PolygonUtils::findClosest(unsupported_location, current_outlines);
    Point node_location = cpp.p();

    std::shared_ptr<LightningTreeNode> sub_tree(nullptr);
    coord_t current_dist = getWeightedDistance(node_location, unsupported_location);
    auto candidate_trees = tree_node_locator.getNearbyVals(unsupported_location, std::min(current_dist, supporting_radius));
    for (auto& candidate_wptr : candidate_trees)
    {
        auto candidate_sub_tree = candidate_wptr.lock();
        if (candidate_sub_tree && candidate_sub_tree != exclude_tree && ! (exclude_tree && exclude_tree->hasOffspring(candidate_sub_tree)))
        {
            const coord_t candidate_dist = candidate_sub_tree->getWeightedDistance(unsupported_location, supporting_radius);
            if (candidate_dist < current_dist)
            {
                current_dist = candidate_dist;
                sub_tree = candidate_sub_tree;
            }
        }
    }

    if ( ! sub_tree)
    {
        return GroundingLocation{nullptr, cpp};
    }
    else
    {
        return GroundingLocation{sub_tree, std::optional<ClosestPolygonPoint>()};
    }
}

void LightningLayer::attach(const Point& unsupported_location, const GroundingLocation& grounding_loc)
{
    // Update trees & distance fields.
    if (grounding_loc.boundary_location)
    {
        tree_roots.insert(LightningTreeNode::create(grounding_loc.p(), unsupported_location));
    }
    else
    {
        grounding_loc.tree_node->addChild(unsupported_location);
    }
}

void LightningLayer::reconnectRoots(std::unordered_set<std::shared_ptr<LightningTreeNode>>& to_be_reconnected_tree_roots, const Polygons& current_outlines, const coord_t supporting_radius)
{
    constexpr coord_t locator_cell_size = 2000;
    SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>> tree_node_locator(locator_cell_size);
    fillLocator(tree_node_locator);

    const LightningTreeNode::node_visitor_func_t add_node_to_locator_func = getAddToLocatorFunc(tree_node_locator);
    for (auto root_ptr : to_be_reconnected_tree_roots)
    {
        GroundingLocation ground = getBestGroundingLocation(root_ptr->getLocation(), current_outlines, supporting_radius, tree_node_locator, root_ptr);
        if (ground.boundary_location)
        {
            if (ground.boundary_location.value().p() == root_ptr->getLocation())
            {
                continue; // Already on the boundary.
            }

            auto new_root = LightningTreeNode::create(ground.p());
            new_root->addChild(root_ptr);

            tree_roots.insert(new_root);
        }
        else
        {
            assert(ground.tree_node);
            assert(ground.tree_node != root_ptr);
            assert( ! root_ptr->hasOffspring(ground.tree_node));
            assert( ! ground.tree_node->hasOffspring(root_ptr));

            ground.tree_node->addChild(root_ptr);
        }
        tree_roots.erase(root_ptr);
    }
}

const LightningLayer& LightningGenerator::getTreesForLayer(const size_t& layer_id)
{
    assert(layer_id < lightning_layers.size());
    return lightning_layers[layer_id];
}

// Returns 'added someting'.
Polygons LightningLayer::convertToLines() const
{
    Polygons result_lines;
    if (tree_roots.empty())
    {
        return result_lines;
    }

    // TODO: The convert trees to lines 'algorithm' is way too simple right now (unless they're already going to be connected later).
    LightningTreeNode::branch_visitor_func_t convert_trees_to_lines =
        [&result_lines](const Point& node, const Point& leaf)
        {
            result_lines.addLine(node, leaf);
        };
    for (const auto& tree : tree_roots)
    {
        tree->visitBranches(convert_trees_to_lines);
    }
    return result_lines;
}

// -- -- -- -- -- --
// -- -- -- -- -- --


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
