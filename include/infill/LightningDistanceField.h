// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_DISTANCE_FIELD_H
#define LIGHTNING_DISTANCE_FIELD_H

#include "../utils/SquareGrid.h" //Tracking for each location the distance to overhang.
#include "geometry/Polygon.h" //Using outlines to fill and tracking overhang.

namespace cura
{

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
    LightningDistanceField(const coord_t& radius, const Shape& current_outline, const Shape& current_overhang);

    /*!
     * Gets the next unsupported location to be supported by a new branch.
     * \param p Output variable for the next point to support.
     * \return ``true`` if successful, or ``false`` if there are no more points
     * to consider.
     */
    bool tryGetNextPoint(Point2LL* p) const;

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
    void update(const Point2LL& to_node, const Point2LL& added_leaf);

protected:
    using GridPoint = SquareGrid::GridPoint;

    /*!
     * Spacing between grid points to consider supporting.
     */
    coord_t cell_size_;

    /*!
     * Grid points to consider supporting, with each point maintaining its
     * distance to the nearest support point.
     */
    SquareGrid grid_;

    /*!
     * The radius of the area of the layer above supported by a point on a
     * branch of a tree.
     */
    coord_t supporting_radius_;

    /*!
     * The total infill area on the current layer.
     */
    const Shape& current_outline_;

    /*!
     * The overhang that gets introduced on this layer, which the infill will
     * need to support.
     */
    const Shape& current_overhang_;

    /*!
     * Represents a small discrete area of infill that needs to be supported.
     */
    struct UnsupCell
    {
        /*!
         * The position of the center of this cell.
         */
        Point2LL loc_;

        /*!
         * How far this cell is removed from the ``current_outline`` polygon,
         * the edge of the infill area.
         */
        coord_t dist_to_boundary_;

        UnsupCell(SquareGrid::GridPoint loc, coord_t dist_to_boundary)
            : loc_(loc)
            , dist_to_boundary_(dist_to_boundary)
        {
        }
    };

    /*!
     * Cells which still need to be supported at some point.
     */
    std::list<UnsupCell> unsupported_points_;

    /*!
     * Links the unsupported points to a grid point, so that we can quickly look
     * up the cell belonging to a certain position in the grid.
     */
    std::unordered_map<SquareGrid::GridPoint, std::list<UnsupCell>::iterator> unsupported_points_grid_;
};

} // namespace cura

#endif // LIGHTNING_DISTANCE_FIELD_H
