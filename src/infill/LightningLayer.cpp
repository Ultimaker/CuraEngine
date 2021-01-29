//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningLayer.h"

#include <iterator> // advance

#include "LightningTree.h"

#include "../sliceDataStorage.h"
#include "../utils/linearAlg2D.h"
#include "../utils/SVG.h"
#include "../utils/SparsePointGridInclusive.h"

using namespace cura;

coord_t LightningLayer::getWeightedDistance(const Point& boundary_loc, const Point& unsupported_loc)
{
    return vSize(boundary_loc - unsupported_loc);
}

PolygonLightningDistanceField::PolygonLightningDistanceField
(
    const coord_t& radius,
 const Polygons& current_outline,
 const Polygons& current_overhang,
 const std::vector<std::shared_ptr<LightningTreeNode>>& initial_trees
)
{
    supporting_radius = radius;
    Polygons supporting_polylines = current_outline;
    for (PolygonRef poly : supporting_polylines)
    {
        if (!poly.empty())
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

bool PolygonLightningDistanceField::tryGetNextPoint(Point* p, coord_t supporting_radius) const
{
    if (unsupported.area() < 25)
    {
        return false;
    }
    coord_t total_length = unsupported[0].polygonLength();
    coord_t dist_to_point_on_boundary = std::rand() % total_length;
    ClosestPolygonPoint cpp = PolygonUtils::walk(ClosestPolygonPoint(unsupported[0][0], 0, unsupported[0]), dist_to_point_on_boundary);
    *p = PolygonUtils::moveInside(cpp, supporting_radius);
    if (!unsupported.inside(*p))
    {
        PolygonUtils::moveInside(unsupported, *p, supporting_radius / 2);
    }
    // NOTE: it's okay for the rare case where a point ends up outside; it's just an inefficient tree branch.
    return true;
}

void PolygonLightningDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    Polygons line;
    line.addLine(to_node, added_leaf);
    Polygons offsetted = line.offsetPolyLine(supporting_radius, ClipperLib::jtRound);
    supported = supported.unionPolygons(offsetted);
    unsupported = unsupported.difference(supported);
}

// -- -- -- -- -- --
// -- -- -- -- -- --


LightningDistanceField::LightningDistanceField
(
    const coord_t& radius,
 const Polygons& current_outline,
 const Polygons& current_overhang,
 const std::vector<std::shared_ptr<LightningTreeNode>>& initial_trees
)
: cell_size(radius / 6) // TODO: make configurable!
, grid(cell_size)
, supporting_radius(radius)
, current_outline(current_outline)
, current_overhang(current_overhang)
{
    std::vector<Point> regular_dots = PolygonUtils::spreadDotsArea(current_overhang, cell_size);
    for (Point p : regular_dots)
    {
        const ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, current_outline);
        const coord_t dist_to_boundary = vSize(p - cpp.p());
        unsupported_points.emplace_back(p, dist_to_boundary);
    }
    unsupported_points.sort(
        [](const UnsupCell& a, const UnsupCell& b)
        {
            coord_t da = a.dist_to_boundary + std::hash<Point>{}(a.loc) % 191;
            coord_t db = b.dist_to_boundary + std::hash<Point>{}(b.loc) % 191;
            if (da == db) return std::hash<Point>{}(a.loc) % 17 < std::hash<Point>{}(b.loc) % 17;
            return da < db;
        });
    for (auto it = unsupported_points.begin(); it != unsupported_points.end(); ++it)
    {
        UnsupCell& cell = *it;
        unsupported_points_grid.emplace(grid.toGridPoint(cell.loc), it);
    }
}

bool LightningDistanceField::tryGetNextPoint(Point* p, coord_t supporting_radius) const
{
    if (unsupported_points.empty()) return false;
    *p = unsupported_points.front().loc;
    return true;
}

void LightningDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    grid.processNearby(added_leaf, supporting_radius,
                       [added_leaf, this](const SquareGrid::GridPoint& grid_loc)
                       {
                           auto it = unsupported_points_grid.find(grid_loc);
                           if (it != unsupported_points_grid.end())
                           {
                               std::list<UnsupCell>::iterator& list_it = it->second;
                               UnsupCell& cell = *list_it;
                               if (shorterThen(cell.loc - added_leaf, supporting_radius))
                               {
                                   unsupported_points.erase(list_it);
                                   unsupported_points_grid.erase(it);
                               }
                           }
                           return true;
                       });
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

void LightningLayer::fillLocator(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator)
{
    const LightningTreeNode::node_visitor_func_t add_node_to_locator_func =
        [&tree_node_locator](std::shared_ptr<LightningTreeNode> node)
        {
            tree_node_locator.insert(node->getLocation(), node);
        };
    for (auto& tree : tree_roots)
    {
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
    while (distance_field.tryGetNextPoint(&unsupported_location, supporting_radius) && i_debug < debug_max_iterations)
    {
        ++i_debug;

        GroundingLocation grounding_loc = getBestGroundingLocation(unsupported_location, current_outlines, supporting_radius, tree_node_locator);

        // TODO: update unsupported_location to lie closer to grounding_loc

        auto tree_node = attach(unsupported_location, grounding_loc);
        tree_node_locator.insert(tree_node->getLocation(), tree_node);

        // update distance field
        distance_field.update(grounding_loc.p(), unsupported_location);
    }
    if (i_debug > 3)
    {
        SVG svg("trees.svg", AABB(current_outlines));
        svg.writePolygons(current_outlines, SVG::Color::GREEN);
        svg.writePolygons(current_overhang, SVG::Color::BLUE);
//         Polygons lines = convertToLines();
//         svg.writePolylines(lines);
        for (auto root : tree_roots)
        {
            root->visitBranches([&svg](const Point& a, const Point& b)
            {
                svg.writeLine(a, b);
            });
        }
        
    }
}

GroundingLocation LightningLayer::getBestGroundingLocation(const Point& unsupported_location, const Polygons& current_outlines, const coord_t supporting_radius, const SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator, const std::shared_ptr<LightningTreeNode>& exclude_tree)
{
    ClosestPolygonPoint cpp = PolygonUtils::findClosest(unsupported_location, current_outlines);
    Point node_location = cpp.p();

    std::shared_ptr<LightningTreeNode> sub_tree(nullptr);
    coord_t current_dist = getWeightedDistance(node_location, unsupported_location);
    if (current_dist >= supporting_radius) // don't reconnect tree roots to other trees if they are already at/near the boundary
    { // TODO: make boundary size in which we ignore the valence rule configurable
        auto candidate_trees = tree_node_locator.getNearbyVals(unsupported_location, std::min(current_dist, supporting_radius));
        for (auto& candidate_wptr : candidate_trees)
        {
            auto candidate_sub_tree = candidate_wptr.lock();
            if (candidate_sub_tree && candidate_sub_tree != exclude_tree && !(exclude_tree && exclude_tree->hasOffspring(candidate_sub_tree)))
            {
                const coord_t candidate_dist = candidate_sub_tree->getWeightedDistance(unsupported_location, supporting_radius);
                if (candidate_dist < current_dist)
                {
                    current_dist = candidate_dist;
                    sub_tree = candidate_sub_tree;
                }
            }
        }
    }

    if (!sub_tree)
    {
        return GroundingLocation{ nullptr, cpp };
    }
    else
    {
        return GroundingLocation{ sub_tree, std::optional<ClosestPolygonPoint>() };
    }
}

std::shared_ptr<LightningTreeNode> LightningLayer::attach(const Point& unsupported_location, const GroundingLocation& grounding_loc)
{
    // Update trees & distance fields.
    if (grounding_loc.boundary_location)
    {
        tree_roots.push_back(LightningTreeNode::create(grounding_loc.p(), unsupported_location));
        return tree_roots.back();
    }
    else
    {
        return grounding_loc.tree_node->addChild(unsupported_location);
    }
}

void LightningLayer::reconnectRoots(std::vector<std::shared_ptr<LightningTreeNode>>& to_be_reconnected_tree_roots, const Polygons& current_outlines, const coord_t supporting_radius)
{
    constexpr coord_t locator_cell_size = 2000;
    SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>> tree_node_locator(locator_cell_size);
    fillLocator(tree_node_locator);

    for (auto root_ptr : to_be_reconnected_tree_roots)
    {
        auto old_root_it = std::find(tree_roots.begin(), tree_roots.end(), root_ptr);
        GroundingLocation ground = getBestGroundingLocation(root_ptr->getLocation(), current_outlines, supporting_radius, tree_node_locator, root_ptr);
        if (ground.boundary_location)
        {
            if (ground.boundary_location.value().p() == root_ptr->getLocation())
            {
                continue; // Already on the boundary.
            }

            auto new_root = LightningTreeNode::create(ground.p());
            new_root->addChild(root_ptr);
            tree_node_locator.insert(new_root->getLocation(), new_root);

            *old_root_it = std::move(new_root); // replace old root with new root
        }
        else
        {
            assert(ground.tree_node);
            assert(ground.tree_node != root_ptr);
            assert(!root_ptr->hasOffspring(ground.tree_node));
            assert(!ground.tree_node->hasOffspring(root_ptr));

            ground.tree_node->addChild(root_ptr);

            // remove old root
            *old_root_it = std::move(tree_roots.back());
            tree_roots.pop_back();
        }
    }
}

// Returns 'added someting'.
Polygons LightningLayer::convertToLines() const
{
    Polygons result_lines;
    if (tree_roots.empty())
    {
        return result_lines;
    }

    for (const auto& tree : tree_roots)
    {
        tree->convertToPolylines(result_lines);
    }

    // TODO: allow for polylines!
    Polygons split_lines;
    for (PolygonRef line : result_lines)
    {
        if (line.size() <= 1) continue;
        Point last = line[0];
        for (size_t point_idx = 1; point_idx < line.size(); point_idx++)
        {
            Point here = line[point_idx];
            split_lines.addLine(last, here);
            last = here;
        }
    }

    return split_lines;
}
