// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/LightningTreeNode.h"

#include "geometry/OpenPolyline.h"
#include "utils/linearAlg2D.h"

using namespace cura;

using LightningTreeNodeSPtr = std::shared_ptr<LightningTreeNode>;

coord_t LightningTreeNode::getWeightedDistance(const Point2LL& unsupported_location, const coord_t& supporting_radius) const
{
    constexpr coord_t min_valence_for_boost = 0;
    constexpr coord_t max_valence_for_boost = 4;
    constexpr coord_t valence_boost_multiplier = 4;

    const size_t valence = (! is_root_) + children_.size();
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
    for (auto& child_ptr : children_)
    {
        if (child_ptr->hasOffspring(to_be_checked))
            return true;
    }
    return false;
}

const Point2LL& LightningTreeNode::getLocation() const
{
    return p_;
}

void LightningTreeNode::setLocation(const Point2LL& loc)
{
    p_ = loc;
}

LightningTreeNodeSPtr LightningTreeNode::addChild(const Point2LL& child_loc)
{
    assert(p_ != child_loc);
    LightningTreeNodeSPtr child = LightningTreeNode::create(child_loc);
    return addChild(child);
}

LightningTreeNodeSPtr LightningTreeNode::addChild(LightningTreeNodeSPtr& new_child)
{
    assert(new_child != shared_from_this());
    // assert(p != new_child->p); // NOTE: No problem for now. Issue to solve later. Maybe even afetr final. Low prio.
    children_.push_back(new_child);
    new_child->parent_ = shared_from_this();
    new_child->is_root_ = false;
    return new_child;
}

void LightningTreeNode::propagateToNextLayer(
    std::vector<LightningTreeNodeSPtr>& next_trees,
    const Shape& next_outlines,
    const LocToLineGrid& outline_locator,
    const coord_t prune_distance,
    const coord_t smooth_magnitude,
    const coord_t max_remove_colinear_dist) const
{
    auto tree_below = deepCopy();

    tree_below->prune(prune_distance);
    tree_below->straighten(smooth_magnitude, max_remove_colinear_dist);
    if (tree_below->realign(next_outlines, outline_locator, next_trees))
    {
        next_trees.push_back(tree_below);
    }
}

// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
void LightningTreeNode::visitBranches(const std::function<void(const Point2LL&, const Point2LL&)>& visitor) const
{
    for (const auto& node : children_)
    {
        assert(node->parent_.lock() == shared_from_this());
        visitor(p_, node->p_);
        node->visitBranches(visitor);
    }
}

// NOTE: Depth-first, as currently implemented.
void LightningTreeNode::visitNodes(const std::function<void(LightningTreeNodeSPtr)>& visitor)
{
    visitor(shared_from_this());
    for (const auto& node : children_)
    {
        assert(node->parent_.lock() == shared_from_this());
        node->visitNodes(visitor);
    }
}

LightningTreeNode::LightningTreeNode(const Point2LL& p, const std::optional<Point2LL>& last_grounding_location /*= std::nullopt*/)
    : is_root_(true)
    , p_(p)
    , last_grounding_location_(last_grounding_location)
{
}

LightningTreeNodeSPtr LightningTreeNode::deepCopy() const
{
    LightningTreeNodeSPtr local_root = LightningTreeNode::create(p_);
    local_root->is_root_ = is_root_;
    if (is_root_)
    {
        local_root->last_grounding_location_ = last_grounding_location_.value_or(p_);
    }
    local_root->children_.reserve(children_.size());
    for (const auto& node : children_)
    {
        LightningTreeNodeSPtr child = node->deepCopy();
        child->parent_ = local_root;
        local_root->children_.push_back(child);
    }
    return local_root;
}

void LightningTreeNode::reroot(LightningTreeNodeSPtr new_parent /*= nullptr*/)
{
    if (! is_root_)
    {
        auto old_parent = parent_.lock();
        old_parent->reroot(shared_from_this());
        children_.push_back(old_parent);
    }

    if (new_parent)
    {
        children_.erase(std::remove(children_.begin(), children_.end(), new_parent), children_.end());
        is_root_ = false;
        parent_ = new_parent;
    }
    else
    {
        is_root_ = true;
        parent_.reset();
    }
}

LightningTreeNodeSPtr LightningTreeNode::closestNode(const Point2LL& loc)
{
    LightningTreeNodeSPtr result = shared_from_this();
    coord_t closest_dist2 = vSize2(p_ - loc);

    for (const auto& child : children_)
    {
        LightningTreeNodeSPtr candidate_node = child->closestNode(loc);
        const coord_t child_dist2 = vSize2(candidate_node->p_ - loc);
        if (child_dist2 < closest_dist2)
        {
            closest_dist2 = child_dist2;
            result = candidate_node;
        }
    }

    return result;
}

bool LightningTreeNode::realign(const Shape& outlines, const LocToLineGrid& outline_locator, std::vector<LightningTreeNodeSPtr>& rerooted_parts)
{
    if (outlines.empty())
    {
        return false;
    }

    if (outlines.inside(p_, true))
    {
        // Only keep children that have an unbroken connection to here, realign will put the rest in rerooted parts due to recursion:
        Point2LL coll;
        bool reground_me = false;
        const auto remove_unconnected_func{
            [&](const LightningTreeNodeSPtr& child)
            {
                bool connect_branch = child->realign(outlines, outline_locator, rerooted_parts);
                if (connect_branch && PolygonUtils::lineSegmentPolygonsIntersection(child->p_, p_, outlines, outline_locator, coll, outline_locator.getCellSize() * 2))
                {
                    child->last_grounding_location_.reset();
                    child->parent_.reset();
                    child->is_root_ = true;
                    rerooted_parts.push_back(child);

                    reground_me = true;
                    connect_branch = false;
                }
                return ! connect_branch;
            }
        };
        children_.erase(std::remove_if(children_.begin(), children_.end(), remove_unconnected_func), children_.end());
        if (reground_me)
        {
            last_grounding_location_.reset();
        }
        return true;
    }

    // 'Lift' any decendants out of this tree:
    for (auto& child : children_)
    {
        if (child->realign(outlines, outline_locator, rerooted_parts))
        {
            child->last_grounding_location_ = p_;
            child->parent_.reset();
            child->is_root_ = true;
            rerooted_parts.push_back(child);
        }
    }
    children_.clear();

    return false;
}

void LightningTreeNode::straighten(const coord_t magnitude, const coord_t max_remove_colinear_dist)
{
    straighten(magnitude, p_, 0, max_remove_colinear_dist * max_remove_colinear_dist);
}

LightningTreeNode::RectilinearJunction
    LightningTreeNode::straighten(const coord_t magnitude, const Point2LL& junction_above, const coord_t accumulated_dist, const coord_t max_remove_colinear_dist2)
{
    constexpr coord_t junction_magnitude_factor_numerator = 3;
    constexpr coord_t junction_magnitude_factor_denominator = 4;

    const coord_t junction_magnitude = magnitude * junction_magnitude_factor_numerator / junction_magnitude_factor_denominator;
    if (children_.size() == 1)
    {
        auto child_p = children_.front();
        coord_t child_dist = vSize(p_ - child_p->p_);
        RectilinearJunction junction_below = child_p->straighten(magnitude, junction_above, accumulated_dist + child_dist, max_remove_colinear_dist2);
        coord_t total_dist_to_junction_below = junction_below.total_recti_dist;
        Point2LL a = junction_above;
        Point2LL b = junction_below.junction_loc;
        if (a != b) // should always be true!
        {
            Point2LL ab = b - a;
            Point2LL destination = a + ab * accumulated_dist / std::max(coord_t(1), total_dist_to_junction_below);
            if (shorterThen(destination - p_, magnitude))
            {
                p_ = destination;
            }
            else
            {
                p_ = p_ + normal(destination - p_, magnitude);
            }
        }
        { // remove nodes on linear segments
            constexpr coord_t close_enough = 10;

            child_p = children_.front(); // recursive call to straighten might have removed the child
            const LightningTreeNodeSPtr& parent_node = parent_.lock();
            if (parent_node && vSize2(child_p->p_ - parent_node->p_) < max_remove_colinear_dist2
                && LinearAlg2D::getDist2FromLineSegment(parent_node->p_, p_, child_p->p_) < close_enough)
            {
                child_p->parent_ = parent_;
                for (auto& sibling : parent_node->children_)
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
        Point2LL junction_moving_dir = normal(junction_above - p_, weight);
        bool prevent_junction_moving = false;
        for (auto& child_p : children_)
        {
            const coord_t child_dist = vSize(p_ - child_p->p_);
            RectilinearJunction below = child_p->straighten(magnitude, p_, child_dist, max_remove_colinear_dist2);

            junction_moving_dir += normal(below.junction_loc - p_, weight);
            if (below.total_recti_dist < magnitude) // TODO: make configurable?
            {
                prevent_junction_moving = true; // prevent flipflopping in branches due to straightening and junctoin moving clashing
            }
        }
        if (junction_moving_dir != Point2LL(0, 0) && ! children_.empty() && ! is_root_ && ! prevent_junction_moving)
        {
            coord_t junction_moving_dir_len = vSize(junction_moving_dir);
            if (junction_moving_dir_len > junction_magnitude)
            {
                junction_moving_dir = junction_moving_dir * junction_magnitude / junction_moving_dir_len;
            }
            p_ += junction_moving_dir;
        }
        return RectilinearJunction{ accumulated_dist, p_ };
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
    for (auto child_it = children_.begin(); child_it != children_.end();)
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
            const Point2LL a = getLocation();
            const Point2LL b = child->getLocation();
            const Point2LL ba = a - b;
            const coord_t ab_len = vSize(ba);
            if (dist_pruned_child + ab_len <= pruning_distance)
            { // we're still in the process of pruning
                assert(child->children_.empty() && "when pruning away a node all it's children must already have been pruned away");
                max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child + ab_len);
                child_it = children_.erase(child_it);
            }
            else
            { // pruning stops in between this node and the child
                const Point2LL n = b + normal(ba, pruning_distance - dist_pruned_child);
                assert(std::abs(vSize(n - b) + dist_pruned_child - pruning_distance) < 10 && "total pruned distance must be equal to the pruning_distance");
                max_distance_pruned = std::max(max_distance_pruned, pruning_distance);
                child->setLocation(n);
                ++child_it;
            }
        }
    }

    return max_distance_pruned;
}

const std::optional<Point2LL>& LightningTreeNode::getLastGroundingLocation() const
{
    return last_grounding_location_;
}

void LightningTreeNode::convertToPolylines(OpenLinesSet& output, const coord_t line_width) const
{
    OpenLinesSet result;
    result.emplace_back();
    convertToPolylines(0, result);
    removeJunctionOverlap(result, line_width);
    output.push_back(result);
}

void LightningTreeNode::convertToPolylines(size_t long_line_idx, OpenLinesSet& output) const
{
    if (children_.empty())
    {
        output[long_line_idx].push_back(p_);
        return;
    }
    size_t first_child_idx = rand() % children_.size();
    children_[first_child_idx]->convertToPolylines(long_line_idx, output);
    output[long_line_idx].push_back(p_);

    for (size_t idx_offset = 1; idx_offset < children_.size(); idx_offset++)
    {
        size_t child_idx = (first_child_idx + idx_offset) % children_.size();
        const LightningTreeNode& child = *children_[child_idx];
        output.emplace_back();
        size_t child_line_idx = output.size() - 1;
        child.convertToPolylines(child_line_idx, output);
        output[child_line_idx].push_back(p_);
    }
}

void LightningTreeNode::removeJunctionOverlap(OpenLinesSet& result_lines, const coord_t line_width) const
{
    const coord_t reduction = line_width / 2; // TODO make configurable?
    for (auto poly_it = result_lines.begin(); poly_it != result_lines.end();)
    {
        OpenPolyline& polyline = *poly_it;
        if (polyline.size() <= 1)
        {
            polyline = std::move(result_lines.back());
            result_lines.pop_back();
            continue;
        }

        coord_t to_be_reduced = reduction;
        Point2LL a = polyline.back();
        for (int point_idx = polyline.size() - 2; point_idx >= 0; point_idx--)
        {
            const Point2LL b = polyline[point_idx];
            const Point2LL ab = b - a;
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
