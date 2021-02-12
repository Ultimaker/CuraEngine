//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_LAYER_H
#define LIGHTNING_LAYER_H

#include "../utils/polygonUtils.h"
#include "../utils/SquareGrid.h"

#include <memory>
#include <vector>
#include <list>
#include <unordered_map>

namespace cura
{
class LightningTreeNode;

class LightningDistanceField
{
public:
    /*!
     * constructor
     */
    LightningDistanceField
    (
        const coord_t& radius,
     const Polygons& current_outline,
     const Polygons& current_overhang,
     const std::vector<std::shared_ptr<LightningTreeNode>>& initial_trees
    );
    
    /*!
     * Gets the next unsupported location to be supported by a new branch.
     *
     * Returns false if there are no more points to consider
     */
    bool tryGetNextPoint(Point* p) const;

    /*! update the distance field with a newly added branch
     * TODO: check whether this explanation is correct
     */
    void update(const Point& to_node, const Point& added_leaf);

    /*!
     * Get a new unsupported point, close the already known \p fall_back, which is also close to the nearby tree branch location \p p.
     * 
     * Gets a random point on the boundary of the unsupported cells near \p fall_back,
     * and takes an offset to as far away as \p total_radius,
     * and returns the cell point at that location.
     *    .  .  .  .  .     . = unsupported cell point
     *    .  .  o  .  .     o = offsetted point at total radius; the returned point.
     *    .  .  .  .  .
     *    .  .  .  .  .
     *    .  r  .  .  .     r = random point at the boundary of the unsupported, but close enough to f as \p supporting_radius.
     *          .  .  .
     *             f        f = \p fall_back.
     *    p                 p = \p p.
     * 
     * If nothing is found, return \p fall_back instead.
     */
    Point getNearbyUnsupportedPoint(const Point p, const Point fall_back, coord_t supporting_radius, coord_t total_radius) const;
protected:
    using GridPoint = SquareGrid::GridPoint;
    coord_t cell_size;
    SquareGrid grid;
    coord_t supporting_radius; //!< The radius of the area of the layer above supported by a point on a branch of a tree
    
    
    const Polygons& current_outline;
    const Polygons& current_overhang;
    
    
    struct UnsupCell
    {
        UnsupCell(SquareGrid::GridPoint loc, coord_t dist_to_boundary)
        : loc(loc)
        , dist_to_boundary(dist_to_boundary)
        {}
        Point loc;
        coord_t dist_to_boundary;
    };
    std::list<UnsupCell> unsupported_points;
    std::unordered_map<SquareGrid::GridPoint, std::list<UnsupCell>::iterator> unsupported_points_grid;
};


struct GroundingLocation
{
    std::shared_ptr<LightningTreeNode> tree_node; //!< not null if the gounding location is on a tree
    std::optional<ClosestPolygonPoint> boundary_location; //!< in case the gounding location is on the boundary
    coord_t weighted_distance;
    Point p() const;
};

/*!
 * A layer of the lightning fill.
 *
 * Contains the trees to be printed and propagated to the next layer below.
 */
class LightningLayer
{
public:
    std::vector<std::shared_ptr<LightningTreeNode>> tree_roots;

    void generateNewTrees(const Polygons& current_overhang, Polygons& current_outlines, coord_t supporting_radius);

    /*! Determine & connect to connection point in tree/outline.
     * \param min_dist_from_boundary_for_tree If the unsupported point is closer to the boundary than this then don't consider connecting it to a tree
     */
    GroundingLocation getBestGroundingLocation(const Point& unsupported_location, const Polygons& current_outlines, const coord_t supporting_radius, const coord_t min_dist_from_boundary_for_tree, const SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator, const std::shared_ptr<LightningTreeNode>& exclude_tree = nullptr);

    /*!
     * 
     * \param[out] new_child The new child node introduced
     * \param[out] new_root The new root node if one had been made
     * \return Whether a new root was added
     */
    bool attach(const Point& unsupported_loc, const GroundingLocation& ground, std::shared_ptr<LightningTreeNode>& new_child, std::shared_ptr<LightningTreeNode>& new_root);

    void reconnectRoots(std::vector<std::shared_ptr<LightningTreeNode>>& to_be_reconnected_tree_roots, const Polygons& current_outlines, const coord_t supporting_radius, const coord_t wall_supporting_radius, const coord_t prune_length);

    Polygons convertToLines(const coord_t line_width) const;

    coord_t getWeightedDistance(const Point& boundary_loc, const Point& unsupported_loc);

    void fillLocator(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator);
};

} // namespace cura

#endif // LIGHTNING_LAYER_H
