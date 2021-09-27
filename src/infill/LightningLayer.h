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

/*!
 * 2D field that maintains locations which need to be supported for Lightning
 * Infill.
 *
 * This field contains a set of "cells", spaced out in a grid. Each cell
 * maintains how far it is removed from the edge, which is used to determine
 * how it gets supported by Lightning Infill.
 */
class LightningDistanceField
{
public:
    /*!
     * Construct a new field to calculate Lightning Infill with.
     * \param radius The radius of influence that an infill line is expected to
     * support in the layer above.
     * \param current_outline The total infill area on this layer.
     * \param current_overhang The overhang that needs to be supported on this
     * layer.
     */
    LightningDistanceField(const coord_t& radius, const Polygons& current_outline, const Polygons& current_overhang);
    
    /*!
     * Gets the next unsupported location to be supported by a new branch.
     * \param p Output variable for the next point to support.
     * \return ``true`` if successful, or ``false`` if there are no more points
     * to consider.
     */
    bool tryGetNextPoint(Point* p) const;

    /*!
     * Update the distance field with a newly added branch.
     *
     * The branch is a line extending from \p to_node to \p added_leaf . This
     * function updates the grid cells so that the distance field knows how far
     * off it is from being supported by the current pattern. Grid points are
     * updated with sampling points spaced out by the supporting radius along
     * the line.
     * \param to_node The node endpoint of the newly added branch.
     * \param added_leaf The location of the leaf of the newly added branch,
     * drawing a straight line to the node.
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

    /*!
     * Spacing between grid points to consider supporting.
     */
    coord_t cell_size;

    /*!
     * Grid points to consider supporting, with each point maintaining its
     * distance to the nearest support point.
     */
    SquareGrid grid;

    /*!
     * The radius of the area of the layer above supported by a point on a
     * branch of a tree.
     */
    coord_t supporting_radius;

    /*!
     * The total infill area on the current layer.
     */
    const Polygons& current_outline;

    /*!
     * The overhang that gets introduced on this layer, which the infill will
     * need to support.
     */
    const Polygons& current_overhang;

    /*!
     * Represents a small discrete area of infill that needs to be supported.
     */
    struct UnsupCell
    {
        UnsupCell(SquareGrid::GridPoint loc, coord_t dist_to_boundary)
        : loc(loc)
        , dist_to_boundary(dist_to_boundary)
        {}

        /*!
         * The position of the center of this cell.
         */
        Point loc;

        /*!
         * How far this cell is removed from the ``current_outline`` polygon,
         * the edge of the infill area.
         */
        coord_t dist_to_boundary;
    };

    /*!
     * Cells which still need to be supported at some point.
     */
    std::list<UnsupCell> unsupported_points;

    /*!
     * Links the unsupported points to a grid point, so that we can quickly look
     * up the cell belonging to a certain position in the grid.
     */
    std::unordered_map<SquareGrid::GridPoint, std::list<UnsupCell>::iterator> unsupported_points_grid;
};


struct GroundingLocation
{
    std::shared_ptr<LightningTreeNode> tree_node; //!< not null if the gounding location is on a tree
    std::optional<ClosestPolygonPoint> boundary_location; //!< in case the gounding location is on the boundary
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

    void reconnectRoots(std::vector<std::shared_ptr<LightningTreeNode>>& to_be_reconnected_tree_roots, const Polygons& current_outlines, const coord_t supporting_radius, const coord_t wall_supporting_radius);

    Polygons convertToLines(const coord_t line_width) const;

    coord_t getWeightedDistance(const Point& boundary_loc, const Point& unsupported_loc);

    void fillLocator(SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>& tree_node_locator);
};

} // namespace cura

#endif // LIGHTNING_LAYER_H
