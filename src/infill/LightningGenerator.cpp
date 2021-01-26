//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningGenerator.h"

#include "../sliceDataStorage.h"
#include "../utils/polygonUtils.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SparsePointGridInclusive.h"

#include <functional>
#include <memory>
#include <vector>

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

coord_t LightningTreeNode::getWeightedDistance(const Point& unsupported_loc, const coord_t& supporting_radius) const
{
    size_t valence = ( ! is_root) + children.size();
    coord_t boost = (0 < valence && valence < 4)? 4 * supporting_radius : 0;
    return vSize(getLocation() - unsupported_loc) - boost;
}


bool LightningTreeNode::hasOffspring(const std::shared_ptr<LightningTreeNode>& to_be_checked) const
{
    if (to_be_checked == shared_from_this()) return true;
    for (auto child_ptr : children)
    {
        if (child_ptr->hasOffspring(to_be_checked)) return true;
    }
    return false;
}

const Point& LightningTreeNode::getLocation() const
{
    return p;
}

void LightningTreeNode::setLocation(const Point& loc)
{
    p = loc;
}

void LightningTreeNode::addChild(const Point& child_loc)
{
    assert(p != child_loc);
    children.push_back(LightningTreeNode::create(child_loc));
}

void LightningTreeNode::addChild(std::shared_ptr<LightningTreeNode>& new_child)
{
    assert(new_child != shared_from_this());
//     assert(p != new_child->p);
    if (p == new_child->p)
        std::cerr << "wtf\n";
    children.push_back(new_child);
    new_child->is_root = false;
}

std::shared_ptr<LightningTreeNode> LightningTreeNode::findClosestNode(const Point& x, const coord_t& supporting_radius)
{
    coord_t closest_distance = getWeightedDistance(x, supporting_radius);
    std::shared_ptr<LightningTreeNode> closest_node = shared_from_this();
    findClosestNodeHelper(x, supporting_radius, closest_distance, closest_node);
    return closest_node;
}

void LightningTreeNode::propagateToNextLayer
(
    std::unordered_set<std::shared_ptr<LightningTreeNode>>& next_trees,
    const Polygons& next_outlines,
    const coord_t& prune_distance,
    const coord_t& smooth_magnitude
) const
{
    auto tree_below = deepCopy();

    // TODO: What is the correct order of the following operations?
    //       (NOTE: in case realign turns out _not_ to be last, would need to rewrite a few things, see the 'rerooted_parts' parameter of that function).
    tree_below->prune(prune_distance);
    tree_below->straighten(smooth_magnitude);
    if (tree_below->realign(next_outlines, next_trees))
    {
        next_trees.insert(tree_below);
    }
}

// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
void LightningTreeNode::visitBranches(const branch_visitor_func_t& visitor) const
{
    for (const auto& node : children)
    {
        visitor(p, node->p);
        node->visitBranches(visitor);
    }
}

// NOTE: Depth-first, as currently implemented.
void LightningTreeNode::visitNodes(const node_visitor_func_t& visitor)
{
    visitor(shared_from_this());
    for (const auto& node : children)
    {
        node->visitNodes(visitor);
    }
}

// Node:
LightningTreeNode::LightningTreeNode(const Point& p) : is_root(false), p(p) {}

// Root (and Trunk):
LightningTreeNode::LightningTreeNode(const Point& a, const Point& b) : LightningTreeNode(a)
{
    children.push_back(LightningTreeNode::create(b));
    is_root = true;
}

void LightningTreeNode::findClosestNodeHelper(const Point& x, const coord_t supporting_radius, coord_t& closest_distance, std::shared_ptr<LightningTreeNode>& closest_node)
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

std::shared_ptr<LightningTreeNode> LightningTreeNode::deepCopy() const
{
    std::shared_ptr<LightningTreeNode> local_root = LightningTreeNode::create(p);
    local_root->is_root = is_root;
    local_root->children.reserve(children.size());
    for (const auto& node : children)
    {
        local_root->children.push_back(node->deepCopy());
    }
    return local_root;
}

bool LightningTreeNode::realign(const Polygons& outlines, std::unordered_set<std::shared_ptr<LightningTreeNode>>& rerooted_parts, const bool& connected_to_parent)
{
    // TODO: Hole(s) in the _middle_ of a line-segement, not unlikely since reconnect.
    // TODO: Reconnect if not on outline -> plan is: yes, but not here anymore!

    if (outlines.empty())
    {
        return false;
    }

    if (outlines.inside(p, true))
    {
        // Only keep children that have an unbroken connection to here, realign will put the rest in rerooted parts due to recursion:
        const std::function<bool(const std::shared_ptr<LightningTreeNode>& child)> remove_unconnected_func
        (
            [&outlines, &rerooted_parts](const std::shared_ptr<LightningTreeNode>& child)
            {
                constexpr bool argument_with_connected = true;
                return ! child->realign(outlines, rerooted_parts, argument_with_connected);
            }
        );
        children.erase(std::remove_if(children.begin(), children.end(), remove_unconnected_func), children.end());
        return true;
    }

    // 'Lift' any decendants out of this tree:
    for (auto& child : children)
    {
        constexpr bool argument_with_disconnect = false;
        if (child->realign(outlines, rerooted_parts, argument_with_disconnect))
        {
            rerooted_parts.insert(child);
        }
    }
    children.clear();

    if (connected_to_parent)
    {
        // This will now be a (new_ leaf:
        p = PolygonUtils::findClosest(p, outlines).p();
        return true;
    }

    return false;
}

void LightningTreeNode::straighten(const coord_t& magnitude)
{
    straighten(magnitude, p, 0);
}

LightningTreeNode::RectilinearJunction LightningTreeNode::straighten(const coord_t& magnitude, Point junction_above, coord_t accumulated_dist)
{
    if (children.size() == 1)
    {
        auto child_p = children.front();
        coord_t child_dist = vSize(p - child_p->p);
        RectilinearJunction junction_below = child_p->straighten(magnitude, junction_above, accumulated_dist + child_dist);
        coord_t total_dist_to_junction_below = junction_below.total_recti_dist;
        Point a = junction_above;
        Point b = junction_below.junction_loc;
        if (a != b) // should always be true!
        {
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
coord_t LightningTreeNode::prune(const coord_t& pruning_distance)
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
    auto candidate_trees = tree_node_locator.getNearbyVals(node_location, std::min(current_dist, supporting_radius));
    for (auto& candidate_wptr : candidate_trees)
    {
        auto candidate_sub_tree = candidate_wptr.lock();
        if (candidate_sub_tree && candidate_sub_tree != exclude_tree && ! exclude_tree->hasOffspring(candidate_sub_tree) && ! candidate_sub_tree->hasOffspring(exclude_tree))
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
