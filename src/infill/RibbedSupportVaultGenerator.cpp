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
RibbedVaultTree::point_distance_func_t RibbedVaultTree::getPointDistanceFunction()
{
    // NOTE: The following closure is too simple compared to the paper, used during development, and will be reimplemented.
    return point_distance_func_t(
        [](const Point& a, const Point& b)
        {
            return vSize2(b - a);
        });
}

const Point& RibbedVaultTree::getNode() const
{
    return p;
}

void RibbedVaultTree::addNode(const Point& p)
{
    nodes.push_back(std::make_shared<RibbedVaultTree>(p));
}

std::shared_ptr<RibbedVaultTree> RibbedVaultTree::findClosestNode(const Point& x, const point_distance_func_t& heuristic)
{
    coord_t closest_distance = heuristic(p, x);
    std::shared_ptr<RibbedVaultTree> closest_node = shared_from_this();
    findClosestNodeHelper(x, heuristic, closest_distance, closest_node);
    return closest_node;
}

void RibbedVaultTree::initNextLayer
(
    std::vector<std::shared_ptr<RibbedVaultTree>>& next_trees,
    const Polygons& next_outlines,
    const coord_t& prune_distance,
    const float& smooth_magnitude
) const
{
    next_trees.push_back(deepCopy());
    auto& result = next_trees.back();

    // TODO: What is the correct order of the following operations?
    result->prune(prune_distance);
    result->smooth(smooth_magnitude);
    result->realign(next_outlines, next_trees);
}

// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
void RibbedVaultTree::visitBranches(const visitor_func_t& visitor) const
{
    for (const auto& node : nodes)
    {
        visitor(p, node->p);
        node->visitBranches(visitor);
    }
}

// Node:
RibbedVaultTree::RibbedVaultTree(const Point& p) : p(p) {}

// Root (and Trunk):
RibbedVaultTree::RibbedVaultTree(const Point& a, const Point& b) : RibbedVaultTree(a)
{
    nodes.push_back(std::make_shared<RibbedVaultTree>(b));
    is_root = true;
}

void RibbedVaultTree::findClosestNodeHelper(const Point& x, const point_distance_func_t& heuristic, coord_t& closest_distance, std::shared_ptr<RibbedVaultTree>& closest_node)
{
    for (const auto& node : nodes)
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

std::shared_ptr<RibbedVaultTree> RibbedVaultTree::deepCopy() const
{
    std::shared_ptr<RibbedVaultTree> local_root = std::make_shared<RibbedVaultTree>(p);
    local_root->is_root = is_root;
    local_root->nodes.reserve(nodes.size());
    for (const auto& node : nodes)
    {
        local_root->nodes.push_back(node->deepCopy());
    }
    return local_root;
}

void RibbedVaultTree::realign(const Polygons& outlines, std::vector<std::shared_ptr<RibbedVaultTree>>& rerooted_parts)
{
    // NOTE: Is it neccesary to 'reroot' parts further up the tree, or can it just be done from the root onwards
    //       and ignore any further altercations once the outline is crossed (from the outside) for the first time?

    // NOT IMPLEMENTED YET! (See TODO's near the top of the file.)
}

void RibbedVaultTree::smooth(const float& magnitude)
{
    // NOT IMPLEMENTED YET! (See TODO's near the top of the file.)
}

// Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
bool RibbedVaultTree::prune(const coord_t& distance)
{
    if (distance <= 0)
    {
        return false;
    }

    const Point& local_p = p;
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
        [&distance, &local_p](const std::shared_ptr<RibbedVaultTree>& node)
        {
            const coord_t branch_length = vSize(node->p - local_p);
            return node->prune(distance - branch_length) && branch_length < distance;
        }
    ), nodes.end());

    return nodes.empty();
}

// -- -- -- -- -- --
// -- -- -- -- -- --

void RibbedVaultDistanceMeasure::reinit
(
    const coord_t& radius,
    const Polygons& current_outline,
    const Polygons& current_overhang,
    const std::vector<std::shared_ptr<RibbedVaultTree>>& initial_trees
)
{
    r = radius;
    supported = current_outline.tubeShape(r, 0);

    for (const auto& tree : initial_trees)
    {
        const RibbedVaultTree::visitor_func_t add_offset_branch_func =
            [&](const Point& junction, const Point& branch)
            {
                Polygon line;
                line.add(junction);
                line.add(branch);
                supported.add(line.offset(r));
            };
        tree->visitBranches(add_offset_branch_func);
    }
    supported = supported.unionPolygons();
    unsupported = current_overhang.difference(supported);
}

bool RibbedVaultDistanceMeasure::tryGetNextPoint(Point* p) const
{
    if (unsupported.area() < 25)
    {
        return false;
    }
    const Polygons pick_area = supported.offset(r + 2).intersection(unsupported.offset(2 - r));
    if (pick_area.area() < 25)
    {
        return false;
    }
    p[0] = pick_area[0][std::rand() % pick_area[0].size()];
    return true;
}

void RibbedVaultDistanceMeasure::update(const Point& to_node, const Point& added_leaf)
{
    Polygons line;
    line.addLine(to_node, added_leaf);
    supported = supported.unionPolygons(line.offset(r));
    unsupported = unsupported.difference(supported);
}

// -- -- -- -- -- --
// -- -- -- -- -- --


RibbedSupportVaultGenerator::RibbedSupportVaultGenerator(const coord_t& radius, const SliceMeshStorage& mesh) :
    radius(radius),
    overhang_per_layer(mesh.layer_nr_max_filled_layer),
    trees_per_layer(mesh.layer_nr_max_filled_layer)
{
    size_t layer_id = 0;
    for (const auto& layer : mesh.layers)
    {
        layer_id_by_height.insert({layer.printZ, layer_id});
        ++layer_id;
    }
    generateInitialInternalOverhangs(mesh);  // NOTE: Question: How to deal if it's (actual) support that is generated instead of infill?
    generateTrees(mesh);  // NOTE: Ideally, these would not be in the constructor. TODO?: Rewrite 'Generator' as loose functions and perhaps a struct.
}

void RibbedSupportVaultGenerator::getTreesForLayer(const size_t& layer_id, ribbed_vault_layer_trees_t* p_trees)
{
    assert(layer_id < trees_per_layer.size());
    p_trees[0] = trees_per_layer[layer_id];
}

// Returns 'added someting'.
bool RibbedSupportVaultGenerator::convertTreesToLines(const ribbed_vault_layer_trees_t& trees, Polygons& result_lines)
{
    if (trees.empty())
    {
        return false;
    }

    // TODO: The convert trees to lines 'algorithm' is way too simple right now (unless they're already going to be connected later).
    RibbedVaultTree::visitor_func_t convert_trees_to_lines =
        [&result_lines](const Point& node, const Point& leaf)
        {
            result_lines.addLine(node, leaf);
        };
    for (const auto& tree : trees)
    {
        tree->visitBranches(convert_trees_to_lines);
    }
    return true;
}

//  TODO: Proper, actual, version! ... should probably not be here even (only non static because the radius is used now).
//        (Make sure random is chosen when difference between epsilon or completely equal).
Point RibbedSupportVaultGenerator::getClosestOnOutline(const Point& p, const Polygons& pols) const
{
    Polygon p_offset;
    p_offset.add(p);
    p_offset.add(p + Point(0, 1));
    p_offset.add(p + Point(1, 0));
    Polygons temp = p_offset.offset(radius + 5);
    temp = pols.intersection(temp);

    assert(temp.area() != 0);

    return temp[0][std::rand() % temp[0].size()];
}

// Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
void RibbedSupportVaultGenerator::generateInitialInternalOverhangs(const SliceMeshStorage& mesh)
{
    // Quick and dirty, TODO: the real deal?

    std::shared_ptr<Polygons> p_last_outlines = std::shared_ptr<Polygons>(nullptr);
    std::shared_ptr<Polygons> p_current_outlines = std::shared_ptr<Polygons>(nullptr);

    std::for_each(mesh.layers.rbegin(), mesh.layers.rend(),
        [&](const SliceLayer& current_layer)
        {
            if (! p_current_outlines)
            {
                p_last_outlines = std::make_shared<Polygons>();
                p_current_outlines = std::make_shared<Polygons>();
                mesh.layers[mesh.layer_nr_max_filled_layer].getOutlines(*p_current_outlines);
                return;
            }
            std::swap(p_last_outlines, p_current_outlines);
            current_layer.getOutlines(*p_current_outlines);

            const size_t layer_id = layer_id_by_height[current_layer.printZ];
            overhang_per_layer[layer_id] = p_current_outlines->intersection(*p_current_outlines);
        }
    );
}

void RibbedSupportVaultGenerator::generateTrees(const SliceMeshStorage& mesh)
{
    RibbedVaultDistanceMeasure distance_measure;

    const auto& tree_point_dist_func = RibbedVaultTree::getPointDistanceFunction();

    // For-each layer:
    std::for_each(mesh.layers.rbegin(), mesh.layers.rend(),
        [&](const SliceLayer& current_layer)
        {
            const size_t layer_id = layer_id_by_height[current_layer.printZ];
            const Polygons& current_overhang = overhang_per_layer[layer_id];
            const Polygons current_outlines = current_layer.getOutlines();  // TODO: Cache current outline of layer somewhere

            if (layer_id == mesh.layer_nr_max_filled_layer)
            {
                trees_per_layer[layer_id] = std::vector<std::shared_ptr<RibbedVaultTree>>();
            }
            std::vector<std::shared_ptr<RibbedVaultTree>>& current_trees = trees_per_layer[layer_id];

            // Have (next) area in need of support.
            distance_measure.reinit(radius, current_outlines, current_overhang, current_trees);

            constexpr size_t debug_max_iterations = 32;
            size_t i_debug = 0;

            // Until no more points need to be added to support all:
            // Determine next point from tree/outline areas via distance-field
            Point next;
            while(distance_measure.tryGetNextPoint(&next)    && i_debug < debug_max_iterations)
            {

                ++i_debug;

                // Determine & conect to connection point in tree/outline.
                Point node = getClosestOnOutline(next, current_outlines);

                std::shared_ptr<RibbedVaultTree> sub_tree(nullptr);
                coord_t current_dist = tree_point_dist_func(node, next);
                for (auto& tree : current_trees)
                {
                    assert(tree);

                    auto candidate_sub_tree = tree->findClosestNode(next, tree_point_dist_func);
                    const coord_t candidate_dist = tree_point_dist_func(candidate_sub_tree->getNode(), next);
                    if (candidate_dist < current_dist)
                    {
                        current_dist = candidate_dist;
                        sub_tree = candidate_sub_tree;
                    }
                }

                // Update trees & distance fields.
                if (! sub_tree)
                {
                    current_trees.push_back(std::make_shared<RibbedVaultTree>(node, next));
                }
                else
                {
                    sub_tree->addNode(next);
                }
                distance_measure.update(node, next);
            }

            // Initialize trees for next lower layer from the current one.
            if (layer_id == 0)
            {
                return;
            }
            const size_t lower_layer_id = layer_id - 1;
            std::vector<std::shared_ptr<RibbedVaultTree>>& lower_trees = trees_per_layer[lower_layer_id];
            for (auto& tree : current_trees)
            {
                tree->initNextLayer
                (
                    lower_trees,
                    current_outlines,
                    radius,
                    0.1  // TODO: smooth-factor should be parameter! ... or at least not a random OK seeming magic value.
                );
            }
        });
}
