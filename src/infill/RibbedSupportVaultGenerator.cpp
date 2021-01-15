//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RibbedSupportVaultGenerator.h"

#include "../sliceDataStorage.h"

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

// TODO??: Move/make-settable somehow or merge completely with this class (same as next getter).
RibbedVaultTreeNode::point_distance_func_t RibbedVaultTreeNode::getPointDistanceFunction()
{
    // NOTE: The following closure is too simple compared to the paper, used during development, and will be reimplemented.
    return point_distance_func_t(
        [](const Point& a, const Point& b)
        {
            return vSize2(b - a);
        });
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

std::shared_ptr<RibbedVaultTreeNode> RibbedVaultTreeNode::findClosestNode(const Point& x, const point_distance_func_t& heuristic)
{
    coord_t closest_distance = heuristic(p, x);
    std::shared_ptr<RibbedVaultTreeNode> closest_node = shared_from_this();
    findClosestNodeHelper(x, heuristic, closest_distance, closest_node);
    return closest_node;
}

void RibbedVaultTreeNode::propagateToNextLayer
(
    std::vector<std::shared_ptr<RibbedVaultTreeNode>>& next_trees,
    const Polygons& next_outlines,
    const coord_t& prune_distance,
    const float& smooth_magnitude
) const
{
    next_trees.push_back(deepCopy());
    auto& result = next_trees.back();

    // TODO: What is the correct order of the following operations?
    result->prune(prune_distance);
    result->smoothen(smooth_magnitude);
    result->realign(next_outlines, next_trees);
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

void RibbedVaultTreeNode::findClosestNodeHelper(const Point& x, const point_distance_func_t& heuristic, coord_t& closest_distance, std::shared_ptr<RibbedVaultTreeNode>& closest_node)
{
    for (const auto& node : children)
    {
        node->findClosestNodeHelper(x, heuristic, closest_distance, closest_node);
        const coord_t distance = heuristic(node->p, x);
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

void RibbedVaultTreeNode::realign(const Polygons& outlines, std::vector<std::shared_ptr<RibbedVaultTreeNode>>& rerooted_parts)
{
    // NOTE: Is it neccesary to 'reroot' parts further up the tree, or can it just be done from the root onwards
    //       and ignore any further altercations once the outline is crossed (from the outside) for the first time?

    // NOT IMPLEMENTED YET! (See TODO's near the top of the file.)
}

void RibbedVaultTreeNode::smoothen(const float& magnitude)
{
    // NOT IMPLEMENTED YET! (See TODO's near the top of the file.)
}

// Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
coord_t RibbedVaultTreeNode::prune(const coord_t& pruning_distance)
{
    if (pruning_distance <= 0)
    {
        return 0;
    }

    coord_t distance_pruned = 0;
    for (auto child_it = children.begin(); child_it != children.end(); )
    {
        auto& child = *child_it;
        coord_t dist_pruned_child = child->prune(pruning_distance);
        if (dist_pruned_child >= pruning_distance)
        { // pruning is finished for child; dont modify further
            distance_pruned = std::max(distance_pruned, dist_pruned_child);
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
                distance_pruned = std::max(distance_pruned, dist_pruned_child + ab_len);
                child_it = children.erase(child_it);
            }
            else
            { // pruning stops in between this node and the child
                Point n = b + normal(ba, pruning_distance - dist_pruned_child);
                assert(std::abs(vSize(n - b) + dist_pruned_child - pruning_distance) < 10 && "total pruned distance must be equal to the pruning_distance");
                distance_pruned = std::max(distance_pruned, pruning_distance);
                child->setLocation(n);
                ++child_it;
            }
        }
    }

    return distance_pruned;
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
    supported = supported.unionPolygons(line.offset(supporting_radius));
    unsupported = unsupported.difference(supported);
}

// -- -- -- -- -- --
// -- -- -- -- -- --


RibbedSupportVaultGenerator::RibbedSupportVaultGenerator(const coord_t& radius, const SliceMeshStorage& mesh) :
    radius(radius)
{
    size_t layer_id = 0;
    for (const auto& layer : mesh.layers)
    {
        layer_id_by_height.insert({layer.printZ, layer_id});
        ++layer_id;
    }
    generateInitialInternalOverhangs(mesh, radius);
    generateTrees(mesh);  // NOTE: Ideally, these would not be in the constructor. TODO?: Rewrite 'Generator' as loose functions and perhaps a struct.
}

const RibbedVaultLayer& RibbedSupportVaultGenerator::getTreesForLayer(const size_t& layer_id)
{
    assert(tree_roots_per_layer.count(layer_id));
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
    Polygons infill_area_below;
    for (size_t layer_nr = 0; layer_nr < mesh.layers.size(); layer_nr++)
    {
        const SliceLayer& current_layer = mesh.layers[layer_nr];
        Polygons infill_area_here;
        for (auto& part : current_layer.parts)
        {
            infill_area_here.add(part.getOwnInfillArea());
        }

        Polygons overhang = infill_area_here.intersection(infill_area_below.offset(-supporting_radius));

        overhang_per_layer.emplace(layer_nr, overhang);
        infill_area_below = std::move(infill_area_here);
    }
}

void RibbedSupportVaultGenerator::generateTrees(const SliceMeshStorage& mesh)
{
    const auto& tree_point_dist_func = RibbedVaultTreeNode::getPointDistanceFunction();

    // For-each layer from top to bottom:
    std::for_each(mesh.layers.rbegin(), mesh.layers.rend(),
        [&](const SliceLayer& current_layer)
        {
            const size_t layer_id = layer_id_by_height[current_layer.printZ];
            const Polygons& current_overhang = overhang_per_layer[layer_id];
            const Polygons current_outlines = current_layer.getOutlines();  // TODO: Cache current outline of layer somewhere

            if (tree_roots_per_layer.count(layer_id) == 0)
            {
                tree_roots_per_layer.emplace(layer_id, RibbedVaultLayer());
            }
            std::vector<std::shared_ptr<RibbedVaultTreeNode>>& current_trees = tree_roots_per_layer[layer_id].tree_roots;

            // Have (next) area in need of support.
            RibbedVaultDistanceField distance_field(radius, current_outlines, current_overhang, current_trees);

            constexpr size_t debug_max_iterations = 32;
            size_t i_debug = 0;

            // Until no more points need to be added to support all:
            // Determine next point from tree/outline areas via distance-field
            Point next;
            while(distance_measure.tryGetNextPoint(&next)    && i_debug < debug_max_iterations)
            {

                ++i_debug;

                // Determine & conect to connection point in tree/outline.
                ClosestPolygonPoint cpp = PolygonUtils::findClosest(unsupported_location, current_outlines);
                Point node_location = cpp.p();

                std::shared_ptr<RibbedVaultTreeNode> sub_tree(nullptr);
                coord_t current_dist = tree_point_dist_func(node, next);
                for (auto& tree : current_trees)
                {
                    assert(tree);

                    auto candidate_sub_tree = tree->findClosestNode(next, tree_point_dist_func);
                    const coord_t candidate_dist = tree_point_dist_func(candidate_sub_tree->getLocation(), next);
                    if (candidate_dist < current_dist)
                    {
                        current_dist = candidate_dist;
                        sub_tree = candidate_sub_tree;
                    }
                }

                // Update trees & distance fields.
                if (! sub_tree)
                {
                    current_trees.push_back(std::make_shared<RibbedVaultTreeNode>(node, next));
                }
                else
                {
                    sub_tree->addChild(next);
                }
                distance_measure.update(node, next);
            }

            // Initialize trees for next lower layer from the current one.
            if (layer_id == 0)
            {
                return;
            }
            const size_t lower_layer_id = layer_id - 1;
            if (tree_roots_per_layer.count(layer_id) == 0)
            {
                tree_roots_per_layer.emplace(lower_layer_id, RibbedVaultLayer());
            }
            std::vector<std::shared_ptr<RibbedVaultTreeNode>>& lower_trees = tree_roots_per_layer[lower_layer_id].tree_roots;
            for (auto& tree : current_trees)
            {
                tree->propagateToNextLayer
                (
                    lower_trees,
                    current_outlines,
                    radius,
                    0.1  // TODO: smooth-factor should be parameter! ... or at least not a random OK seeming magic value.
                );
            }
        });
}
