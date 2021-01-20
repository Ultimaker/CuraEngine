//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RibbedSupportVaultGenerator.h"

#include "../sliceDataStorage.h"
#include "../utils/polygonUtils.h"
#include "../utils/linearAlg2D.h"

#include <functional>
#include <memory>
#include <vector>

//____v

// TODO: Shortest distance to polygons function, and/or write/reuse distancefield things.
//       Idea maybe something can be done with isolines for the distance function?
// TODO: Properly implement the distance heuristic for 'closest node' (it's now just 'euclidian_sqd_distance'), this is different from the distance field!
// TODO: Implement 'Simplify' ... also heuristic? Also: Should the 'sequence_smooth_func' _really_ accept a vector of Points? (Probably a better way.)
// TODO: Implement 'Truncate' ... is it needed to do it within the tree as well (see note-comment in function itself).
// TODO: The convert trees to lines 'algorithm' is way too simple right now (unless they're already going to be connected later).
// TODO: Lots of smaller TODO's in code itself, put on list!

// Implementation in Infill classes & elsewhere (not here):
// TODO: Outline offset, infill-overlap & perimeter gaps.
// TODO: Frontend, make sure it's enabled for 'actual' infill only (search for infill pattern).

// Stretch Goals:
// TODO: Check Unit Tests & Documentation.
// TODO: Result lines handled by Arachne insread :-)
// TODO: Also generate support instead of just infill (and enable that in frontend).
// TODO: Find a way to parallelize part(s) of it??
// TODO: G2/G3 to make trees properly curved? -> Should be in Arachne, not here, but still ;-)

//____^

using namespace cura;

coord_t RibbedVaultLayer::getWeightedDistance(const Point boundary_loc, const Point unsupported_loc)
{
    return vSize(boundary_loc - unsupported_loc);
}

coord_t RibbedVaultTreeNode::getWeightedDistance(const Point unsupported_loc, const coord_t supporting_radius)
{
    size_t valence = ( ! is_root) + children.size();
    coord_t boost = (0 < valence && valence < 4)? 4 * supporting_radius : 0;
    return vSize(getLocation() - unsupported_loc) - boost;
}

const Point& RibbedVaultTreeNode::getLocation() const
{
    return p;
}

void RibbedVaultTreeNode::setLocation(Point loc)
{
    p = loc;
}

void RibbedVaultTreeNode::addChild(const Point& p)
{
    children.push_back(std::make_shared<RibbedVaultTreeNode>(p));
}

std::shared_ptr<RibbedVaultTreeNode> RibbedVaultTreeNode::findClosestNode(const Point& x, const coord_t supporting_radius)
{
    coord_t closest_distance = getWeightedDistance(x, supporting_radius);
    std::shared_ptr<RibbedVaultTreeNode> closest_node = shared_from_this();
    findClosestNodeHelper(x, supporting_radius, closest_distance, closest_node);
    return closest_node;
}

void RibbedVaultTreeNode::propagateToNextLayer
(
    std::vector<std::shared_ptr<RibbedVaultTreeNode>>& next_trees,
    const Polygons& next_outlines,
    const coord_t& prune_distance,
    const coord_t& smooth_magnitude
) const
{
    auto layer_copy = deepCopy();

    // TODO: What is the correct order of the following operations?
    //       (NOTE: in case realign turns out _not_ to be last, would need to rewrite a few things, see the 'rerooted_parts' parameter of that function).
    layer_copy->prune(prune_distance);
    layer_copy->straighten(smooth_magnitude);
    if (layer_copy->realign(next_outlines, next_trees))
    {
        next_trees.push_back(layer_copy);
    }
}

// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
void RibbedVaultTreeNode::visitBranches(const visitor_func_t& visitor) const
{
    for (const auto& node : children)
    {
        visitor(p, node->p);
        node->visitBranches(visitor);
    }
}

// Node:
RibbedVaultTreeNode::RibbedVaultTreeNode(const Point& p) : p(p) {}

// Root (and Trunk):
RibbedVaultTreeNode::RibbedVaultTreeNode(const Point& a, const Point& b) : RibbedVaultTreeNode(a)
{
    children.push_back(std::make_shared<RibbedVaultTreeNode>(b));
    is_root = true;
}

void RibbedVaultTreeNode::findClosestNodeHelper(const Point& x, const coord_t supporting_radius, coord_t& closest_distance, std::shared_ptr<RibbedVaultTreeNode>& closest_node)
{
    for (const auto& node : children)
    {
        node->findClosestNodeHelper(x, supporting_radius, closest_distance, closest_node);
        const coord_t distance = node->getWeightedDistance(x, supporting_radius);
        if (distance < closest_distance)
        {
            closest_node = node;
            closest_distance = distance;
        }
    }
}

std::shared_ptr<RibbedVaultTreeNode> RibbedVaultTreeNode::deepCopy() const
{
    std::shared_ptr<RibbedVaultTreeNode> local_root = std::make_shared<RibbedVaultTreeNode>(p);
    local_root->is_root = is_root;
    local_root->children.reserve(children.size());
    for (const auto& node : children)
    {
        local_root->children.push_back(node->deepCopy());
    }
    return local_root;
}

bool RibbedVaultTreeNode::realign(const Polygons& outlines, std::vector<std::shared_ptr<RibbedVaultTreeNode>>& rerooted_parts)
{
    // NOTE: Is it neccesary to 'reroot' parts further up the tree, or can it just be done from the root onwards
    //       and ignore any further altercations once the outline is crossed (from the outside) for the first time?

    if (outlines.empty())
    {
        return false;
    }
    else if (outlines.inside(p, true))
    {
        return true;
    }
    else if (children.size() == 1 && outlines.inside(children.front()->p, true))
    {
        p = PolygonUtils::findClosest(p, outlines).p();
        return true;
    }
    else
    {
        for (auto& child : children)
        {
            if (child->realign(outlines, rerooted_parts))
            {
                rerooted_parts.push_back(child);
            }
        }
    }
    return false;
}

void RibbedVaultTreeNode::straighten(const coord_t& magnitude)
{
    straighten(magnitude, p, 0);
}
RibbedVaultTreeNode::RectilinearJunction RibbedVaultTreeNode::straighten(const coord_t& magnitude, Point junction_above, coord_t accumulated_dist)
{
    if (children.size() == 1)
    {
        auto child_p = children.front();
        coord_t child_dist = vSize(p - child_p->p);
        RectilinearJunction junction_below = child_p->straighten(magnitude, junction_above, accumulated_dist + child_dist);
        coord_t total_dist_to_junction_below = junction_below.total_recti_dist;
        Point a = junction_above;
        Point b = junction_below.junction_loc;
        Point ab = b - a;
        Point destination = a + ab * accumulated_dist / total_dist_to_junction_below;
        if (shorterThen(destination - p, magnitude))
        {
            p = destination;
        }
        else
        {
            p = p + normal(destination - p, magnitude);
        }
        return junction_below;
    }
    else
    {
        for (auto child_p : children)
        {
            coord_t child_dist = vSize(p - child_p->p);
            child_p->straighten(magnitude, p, child_dist);
        }
        return RectilinearJunction{accumulated_dist, p};
    }
}

// Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
coord_t RibbedVaultTreeNode::prune(const coord_t& pruning_distance)
{
    if (pruning_distance <= 0)
    {
        return 0;
    }

    coord_t max_distance_pruned = 0;
    for (auto child_it = children.begin(); child_it != children.end(); )
    {
        auto& child = *child_it;
        coord_t dist_pruned_child = child->prune(pruning_distance);
        if (dist_pruned_child >= pruning_distance)
        { // pruning is finished for child; dont modify further
            max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child);
            ++child_it;
        }
        else
        {
            Point a = getLocation();
            Point b = child->getLocation();
            Point ba = a - b;
            coord_t ab_len = vSize(ba);
            if (dist_pruned_child + ab_len <= pruning_distance)
            { // we're still in the process of pruning
                assert(child->children.empty() && "when pruning away a node all it's children must already have been pruned away");
                max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child + ab_len);
                child_it = children.erase(child_it);
            }
            else
            { // pruning stops in between this node and the child
                Point n = b + normal(ba, pruning_distance - dist_pruned_child);
                assert(std::abs(vSize(n - b) + dist_pruned_child - pruning_distance) < 10 && "total pruned distance must be equal to the pruning_distance");
                max_distance_pruned = std::max(max_distance_pruned, pruning_distance);
                child->setLocation(n);
                ++child_it;
            }
        }
    }

    return max_distance_pruned;
}

// -- -- -- -- -- --
// -- -- -- -- -- --

RibbedVaultDistanceField::RibbedVaultDistanceField
(
    const coord_t& radius,
    const Polygons& current_outline,
    const Polygons& current_overhang,
    const std::vector<std::shared_ptr<RibbedVaultTreeNode>>& initial_trees
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

    const RibbedVaultTreeNode::visitor_func_t add_offset_branch_func =
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

bool RibbedVaultDistanceField::tryGetNextPoint(Point* p) const
{
    if (unsupported.area() < 25)
    {
        return false;
    }
    *p = unsupported[0][std::rand() % unsupported[0].size()];
    return true;
}

void RibbedVaultDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    Polygons line;
    line.addLine(to_node, added_leaf);
    Polygons offsetted = line.offsetPolyLine(supporting_radius, ClipperLib::jtRound);
    supported = supported.unionPolygons(offsetted);
    unsupported = unsupported.difference(supported);
}

// -- -- -- -- -- --
// -- -- -- -- -- --


RibbedSupportVaultGenerator::RibbedSupportVaultGenerator(const coord_t& radius, const SliceMeshStorage& mesh) :
    supporting_radius (radius)
{
    generateInitialInternalOverhangs(mesh, radius);
    generateTrees(mesh);  // NOTE: Ideally, these would not be in the constructor. TODO?: Rewrite 'Generator' as loose functions and perhaps a struct.
}

const RibbedVaultLayer& RibbedSupportVaultGenerator::getTreesForLayer(const size_t& layer_id)
{
    assert(layer_id < tree_roots_per_layer.size());
    return tree_roots_per_layer[layer_id];
}

// Returns 'added someting'.
Polygons RibbedVaultLayer::convertToLines() const
{
    Polygons result_lines;
    if (tree_roots.empty())
    {
        return result_lines;
    }

    // TODO: The convert trees to lines 'algorithm' is way too simple right now (unless they're already going to be connected later).
    RibbedVaultTreeNode::visitor_func_t convert_trees_to_lines =
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

// Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
void RibbedSupportVaultGenerator::generateInitialInternalOverhangs(const SliceMeshStorage& mesh, coord_t supporting_radius)
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

void RibbedSupportVaultGenerator::generateNewTrees(const SliceMeshStorage& mesh, size_t layer_id, Polygons& current_outlines)
{
    const Polygons& current_overhang = overhang_per_layer[layer_id];

    RibbedVaultLayer& current_vault_layer = tree_roots_per_layer[layer_id];

    // Have (next) area in need of support.
    RibbedVaultDistanceField distance_field(supporting_radius, current_outlines, current_overhang, current_vault_layer.tree_roots);

    constexpr size_t debug_max_iterations = 9999;
    size_t i_debug = 0;

    // Until no more points need to be added to support all:
    // Determine next point from tree/outline areas via distance-field
    Point unsupported_location;
    while (distance_field.tryGetNextPoint(&unsupported_location)    && i_debug < debug_max_iterations)
    {
        ++i_debug;

        // Determine & conect to connection point in tree/outline.
        ClosestPolygonPoint cpp = PolygonUtils::findClosest(unsupported_location, current_outlines);
        Point node_location = cpp.p();

        std::shared_ptr<RibbedVaultTreeNode> sub_tree(nullptr);
        coord_t current_dist = current_vault_layer.getWeightedDistance(node_location, unsupported_location);
        for (auto& tree : current_vault_layer.tree_roots)
        {
            assert(tree);

            auto candidate_sub_tree = tree->findClosestNode(unsupported_location, supporting_radius);
            const coord_t candidate_dist = candidate_sub_tree->getWeightedDistance(unsupported_location, supporting_radius);
            if (candidate_dist < current_dist)
            {
                current_dist = candidate_dist;
                sub_tree = candidate_sub_tree;
            }
        }

        // Update trees & distance fields.
        if (! sub_tree)
        {
            current_vault_layer.tree_roots.push_back(std::make_shared<RibbedVaultTreeNode>(node_location, unsupported_location));
            distance_field.update(node_location, unsupported_location);
        }
        else
        {
            sub_tree->addChild(unsupported_location);
            distance_field.update(sub_tree->getLocation(), unsupported_location);
        }
    }
}

void RibbedSupportVaultGenerator::generateTrees(const SliceMeshStorage& mesh)
{
    tree_roots_per_layer.resize(mesh.layers.size());

    // For-each layer from top to bottom:
    for (int layer_id = mesh.layers.size() - 1; layer_id >= 0; layer_id--)
    {
        Polygons current_outlines;
        for (const auto& part : mesh.layers[layer_id].parts)
        {
            current_outlines.add(part.getOwnInfillArea());
        }

        generateNewTrees(mesh, layer_id, current_outlines);
        
        // Initialize trees for next lower layer from the current one.
        if (layer_id == 0)
        {
            return;
        }
        RibbedVaultLayer& current_vault_layer = tree_roots_per_layer[layer_id];
        std::vector<std::shared_ptr<RibbedVaultTreeNode>>& current_trees = current_vault_layer.tree_roots;
        std::vector<std::shared_ptr<RibbedVaultTreeNode>>& lower_trees = tree_roots_per_layer[layer_id - 1].tree_roots;
        for (auto& tree : current_trees)
        {
            tree->propagateToNextLayer
            (
                lower_trees,
                current_outlines,
                100, // TODO make pruning distance a separate parameter (ideally also as an anglem from which the tanget is used to compute the actual distance for a given layer)
                supporting_radius / 2 // TODO: should smooth-factor be a bit less tan the supporting radius?
            );
        }
    }
}
