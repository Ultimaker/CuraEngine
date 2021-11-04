//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningTreeNode.h"

#include "../utils/linearAlg2D.h"

using namespace cura;

coord_t LightningTreeNode::getWeightedDistance(const Point& unsupported_location, const coord_t& supporting_radius) const
{
    constexpr coord_t min_valence_for_boost = 0;
    constexpr coord_t max_valence_for_boost = 4;
    constexpr coord_t valence_boost_multiplier = 4;

    const size_t valence = (!is_root) + children.size();
    const coord_t valence_boost = (min_valence_for_boost < valence && valence < max_valence_for_boost) ? valence_boost_multiplier * supporting_radius : 0;
    const coord_t dist_here = vSize(getLocation() - unsupported_location);
    return dist_here - valence_boost;
}

bool LightningTreeNode::hasOffspring(const LightningTreeNodeSPtr& to_be_checked) const
{
    if (to_be_checked == shared_from_this())
    {
        return true;
    }
    for (auto& child_ptr : children)
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

LightningTreeNodeSPtr LightningTreeNode::addChild(const Point& child_loc)
{
    assert(p != child_loc);
    LightningTreeNodeSPtr child = LightningTreeNode::create(child_loc);
    return addChild(child);
}

LightningTreeNodeSPtr LightningTreeNode::addChild(LightningTreeNodeSPtr& new_child)
{
    assert(new_child != shared_from_this());
    //assert(p != new_child->p); // NOTE: No problem for now. Issue to solve later. Maybe even afetr final. Low prio.
    children.push_back(new_child);
    new_child->parent = shared_from_this();
    new_child->is_root = false;
    return new_child;
}

void LightningTreeNode::propagateToNextLayer
(
    std::vector<LightningTreeNodeSPtr>& next_trees,
    const Polygons& next_outlines,
    const LocToLineGrid& outline_locator,
    const coord_t& prune_distance,
    const coord_t& smooth_magnitude
) const
{
    auto tree_below = deepCopy();

    tree_below->prune(prune_distance);
    tree_below->straighten(smooth_magnitude);
    if (tree_below->realign(next_outlines, outline_locator, next_trees))
    {
        next_trees.push_back(tree_below);
    }
}

// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
void LightningTreeNode::visitBranches(const std::function<void(const Point&, const Point&)>& visitor) const
{
    for (const auto& node : children)
    {
        assert(node->parent.lock() == shared_from_this());
        visitor(p, node->p);
        node->visitBranches(visitor);
    }
}

// NOTE: Depth-first, as currently implemented.
void LightningTreeNode::visitNodes(const std::function<void(LightningTreeNodeSPtr)>& visitor)
{
    visitor(shared_from_this());
    for (const auto& node : children)
    {
        assert(node->parent.lock() == shared_from_this());
        node->visitNodes(visitor);
    }
}

LightningTreeNode::LightningTreeNode(const Point& p, const std::optional<Point>& last_grounding_location /*= std::nullopt*/)
: is_root(true)
, p(p)
, last_grounding_location(last_grounding_location)
{}

LightningTreeNodeSPtr LightningTreeNode::deepCopy() const
{
    LightningTreeNodeSPtr local_root = LightningTreeNode::create(p);
    local_root->is_root = is_root;
    if (is_root)
    {
        local_root->last_grounding_location = last_grounding_location.value_or(p);
    }
    local_root->children.reserve(children.size());
    for (const auto& node : children)
    {
        LightningTreeNodeSPtr child = node->deepCopy();
        child->parent = local_root;
        local_root->children.push_back(child);
    }
    return local_root;
}

void LightningTreeNode::reroot(LightningTreeNodeSPtr new_parent /*= nullptr*/)
{
    if (! is_root)
    {
        auto old_parent = parent.lock();
        old_parent->reroot(shared_from_this());
        children.push_back(old_parent);
    }

    if (new_parent)
    {
        children.erase(std::remove(children.begin(), children.end(), new_parent), children.end());
        is_root = false;
        parent = new_parent;
    }
    else
    {
        is_root = true;
        parent.reset();
    }
}

LightningTreeNodeSPtr LightningTreeNode::closestNode(const Point& loc)
{
    LightningTreeNodeSPtr result = shared_from_this();
    coord_t closest_dist2 = vSize2(p - loc);

    for (const auto& child : children)
    {
        LightningTreeNodeSPtr candidate_node = child->closestNode(loc);
        const coord_t child_dist2 = vSize2(candidate_node->p - loc);
        if (child_dist2 < closest_dist2)
        {
            closest_dist2 = child_dist2;
            result = candidate_node;
        }
    }

    return result;
}

bool LightningTreeNode::realign
(
    const Polygons& outlines,
    const LocToLineGrid& outline_locator,
    std::vector<LightningTreeNodeSPtr>& rerooted_parts
)
{
    if (outlines.empty())
    {
        return false;
    }

    if (outlines.inside(p, true))
    {
        // Only keep children that have an unbroken connection to here, realign will put the rest in rerooted parts due to recursion:
        Point coll;
        bool reground_me = false;
        const auto remove_unconnected_func
        {
            [&](const LightningTreeNodeSPtr& child)
            {
                bool connect_branch = child->realign(outlines, outline_locator, rerooted_parts);
                if (connect_branch && PolygonUtils::lineSegmentPolygonsIntersection(child->p, p, outlines, outline_locator, coll))
                {
                    child->last_grounding_location.reset(); // child->last_grounding_location = coll;
                    child->parent.reset();
                    child->is_root = true;
                    rerooted_parts.push_back(child);

                    reground_me = true;
                    connect_branch = false;
                }
                return ! connect_branch;
            }
        };
        children.erase(std::remove_if(children.begin(), children.end(), remove_unconnected_func), children.end());
        if (reground_me)
        {
            last_grounding_location.reset();
        }
        return true;
    }

    // 'Lift' any decendants out of this tree:
    for (auto& child : children)
    {
        if (child->realign(outlines, outline_locator, rerooted_parts))
        {
            child->last_grounding_location = p;
            child->parent.reset();
            child->is_root = true;
            rerooted_parts.push_back(child);
        }
    }
    children.clear();

    return false;
}

void LightningTreeNode::straighten(const coord_t& magnitude)
{
    straighten(magnitude, p, 0);
}

LightningTreeNode::RectilinearJunction LightningTreeNode::straighten
(
    const coord_t& magnitude,
    const Point& junction_above,
    const coord_t accumulated_dist
)
{
    constexpr coord_t junction_magnitude_factor_numerator = 3;
    constexpr coord_t junction_magnitude_factor_denominator = 4;

    const coord_t junction_magnitude = magnitude * junction_magnitude_factor_numerator / junction_magnitude_factor_denominator;
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
            Point destination = a + ab * accumulated_dist / std::max(coord_t(1), total_dist_to_junction_below);
            if (shorterThen(destination - p, magnitude))
            {
                p = destination;
            }
            else
            {
                p = p + normal(destination - p, magnitude);
            }
        }
        { // remove nodes on linear segments
            constexpr coord_t close_enough = 10;

            child_p = children.front(); //recursive call to straighten might have removed the child
            const LightningTreeNodeSPtr& parent_node = parent.lock();
            if (parent_node && LinearAlg2D::getDist2FromLineSegment(parent_node->p, p, child_p->p) < close_enough)
            {
                child_p->parent = parent;
                for (auto& sibling : parent_node->children)
                { // find this node among siblings
                    if (sibling == shared_from_this())
                    {
                        sibling = child_p; // replace this node by child
                        break;
                    }
                }
            }
        }
        return junction_below;
    }
    else
    {
        constexpr coord_t weight = 1000;
        Point junction_moving_dir = normal(junction_above - p, weight);
        bool prevent_junction_moving = false;
        for (auto& child_p : children)
        {
            const coord_t child_dist = vSize(p - child_p->p);
            RectilinearJunction below = child_p->straighten(magnitude, p, child_dist);

            junction_moving_dir += normal(below.junction_loc - p, weight);
            if (below.total_recti_dist < magnitude) // TODO: make configurable?
            {
                prevent_junction_moving = true; // prevent flipflopping in branches due to straightening and junctoin moving clashing
            }
        }
        if (junction_moving_dir != Point(0, 0) && ! children.empty() && ! is_root && ! prevent_junction_moving)
        {
            coord_t junction_moving_dir_len = vSize(junction_moving_dir);
            if (junction_moving_dir_len > junction_magnitude)
            {
                junction_moving_dir = junction_moving_dir * junction_magnitude / junction_moving_dir_len;
            }
            p += junction_moving_dir;
        }
        return RectilinearJunction{ accumulated_dist, p };
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
            const Point a = getLocation();
            const Point b = child->getLocation();
            const Point ba = a - b;
            const coord_t ab_len = vSize(ba);
            if (dist_pruned_child + ab_len <= pruning_distance)
            { // we're still in the process of pruning
                assert(child->children.empty() && "when pruning away a node all it's children must already have been pruned away");
                max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child + ab_len);
                child_it = children.erase(child_it);
            }
            else
            { // pruning stops in between this node and the child
                const Point n = b + normal(ba, pruning_distance - dist_pruned_child);
                assert(std::abs(vSize(n - b) + dist_pruned_child - pruning_distance) < 10 && "total pruned distance must be equal to the pruning_distance");
                max_distance_pruned = std::max(max_distance_pruned, pruning_distance);
                child->setLocation(n);
                ++child_it;
            }
        }
    }

    return max_distance_pruned;
}

const std::optional<Point>& LightningTreeNode::getLastGroundingLocation() const
{
    return last_grounding_location;
}

void LightningTreeNode::convertToPolylines(Polygons& output, const coord_t line_width) const
{
    Polygons result;
    result.newPoly();
    convertToPolylines(0, result);
    removeJunctionOverlap(result, line_width);
    output.add(result);
}

void LightningTreeNode::convertToPolylines(size_t long_line_idx, Polygons& output) const
{
    if (children.empty())
    {
        output[long_line_idx].add(p);
        return;
    }
    size_t first_child_idx = rand() % children.size();
    children[first_child_idx]->convertToPolylines(long_line_idx, output);
    output[long_line_idx].add(p);

    for (size_t idx_offset = 1; idx_offset < children.size(); idx_offset++)
    {
        size_t child_idx = (first_child_idx + idx_offset) % children.size();
        const LightningTreeNode& child = *children[child_idx];
        output.newPoly();
        size_t child_line_idx = output.size() - 1;
        child.convertToPolylines(child_line_idx, output);
        output[child_line_idx].add(p);
    }
}

void LightningTreeNode::removeJunctionOverlap(Polygons& result_lines, const coord_t line_width) const
{
    const coord_t reduction = line_width / 2; // TODO make configurable?
    for (auto poly_it = result_lines.begin(); poly_it != result_lines.end(); )
    {
        PolygonRef polyline = *poly_it;
        if (polyline.size() <= 1)
        {
            polyline = std::move(result_lines.back());
            result_lines.pop_back();
            continue;
        }

        coord_t to_be_reduced = reduction;
        Point a = polyline.back();
        for (int point_idx = polyline.size() - 2; point_idx >= 0; point_idx--)
        {
            const Point b = polyline[point_idx];
            const Point ab = b - a;
            const coord_t ab_len = vSize(ab);
            if (ab_len >= to_be_reduced)
            {
                polyline.back() = a + ab * to_be_reduced / ab_len;
                break;
            }
            else
            {
                to_be_reduced -= ab_len;
                polyline.pop_back();
            }
            a = b;
        }

        if (polyline.size() <= 1)
        {
            polyline = std::move(result_lines.back());
            result_lines.pop_back();
        }
        else
        {
            ++poly_it;
        }
    }
}
