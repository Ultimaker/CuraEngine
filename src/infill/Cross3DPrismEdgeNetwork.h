/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_CROSS_3D_PRISM_EDGE_H
#define INFILL_CROSS_3D_PRISM_EDGE_H

#include <array>

#include "../utils/Point3.h"
#include "Cross3D.h"
#include "Cross3DPrism.h"

namespace cura
{


/*!
 * Data structure storing the edge locations of the Cross3D surface.
 * The edge locations are the edges of the vertex positions across all possible layer heights.
 * 
 * These are the raw edge locations without considering overlap prevention.
 * 
 * All edges are added only to the mapping of one of the two neighboring cells:
 * the one with the highest recursion depth
 * or otherwise the left one.
 * 
 * This edges of the space filling surfaces are altered so that the space filling curve is everywhere continuous.
 * 
 * The overlap contraint is not taken into account at this stage.
 * 
 * An edge is all possible positions of a crossfill vertex which is moving over Z along the interface between two prisms.
 * The sides of the prism which are crossed by the crossfill surface are crossed by the vertex in an oscillating manner.
 * The oscillation pattern needs to be adjusted when it would otherwise cause discontinuity:
 *  ____              ____           .
 * [-.  ]            [-.  ]          .
 * [__'-]___         [__'-]___       .
 * [      .-]   ==>  [    '.  ]      .
 * [   .-'  ]        [   .-'  ]      .
 * [.-'_____]        [.-'_____]      .
 *                                   .
 * alternative ascii art:            .
 *                                   .
 * ] /[ [          ] /[ [            .
 * ]/ [ [          ]/ [ [            .
 * ]\ [ [          ]\ [ [            .
 * ] \[ [          ] \[ [            .
 * ]   /[ >change> ]  \ [            .
 * ]  / [          ]  / [            .
 * ] /  [          ] /  [            .
 * ]/   [          ]/   [            .
 * ]\   [    ==>   ]\   [            .
 * ] \  [          ] \  [            .
 * ]  \ [          ]  \ [            .
 * ]   \[          ]   \[            .
 * ]   /[          ]   /[            .
 * ]  / [          ]  / [            .
 * ] /  [          ] /  [            .
 * ]/   [          ]/   [            .
 * 
 * The edge along the side of a prism is represented by a vector of points.
 * 
 * The most difficult problem is when cells are changing density over both XY and Z:
 * Top view:
 *           /|\                        /|\                                                                     .
 *         /  |  \                    /  |  \                                                                   .
 *       /    |    \                /    |    \                                                                 .
 *     /      |      \     ==>    /      |      \                      Problem 1 (not really dealt with here because it's the overlap constraint doing this)
 *   /_______↗|↘       \        /        |        \                    >> sudden jump from just above the middle to on it                .
 *   \       ↑|↓       / \      \       ↗|↘_        \                                                                            .
 *     \     ↑|↓     /     \      \     ↑|   '↘_      \                Problem 2                                                       .
 *       \   ↑|↓   /         \      \   ↑|       '↘_    \              >> sudden jump from a corner to a straight shortcutting line    .
 *         \ ↑|↘ / _ _ _ _ _ _ \      \ ↑|           '↘_  \            = discontinuity along the Z axis                             .
 *           \|/______________ ↘ \      \|_______________ ↘ \                                                      .
 * Solution:
 *           /|\                        /|\                                                             .
 *         /  |  \                    /  |  \                                                           .
 *       /    |    \                /    |    \                                                         .
 *     /      |      \     ==>    /      |      \                                                       .
 *   /_______↗|↘       \        /        |        \                                                     .
 *   \       ↑|  ↘     / \              ↗|↘_        \                                                   .
 *     \     ↑|    ↘_/     \      \     ↑|   '↘_      \                                                 .
 *       \   ↑|    /  '↘_    \      \   ↑|       '↘_    \                                               .
 *         \ ↑|  /        '↘_  \      \ ↑|           '↘_  \                                             .
 *           \|/______________ ↘ \      \|_______________ ↘ \                                           .
 *
 * 
 * Internally we map cells to the edges, which are stored as vectors of Point,
 * but because an edge is always in between two prisms,
 * we only map the prism with the higher density (or the left one) to the edge.
 * We say that that cell is the *owner* of the edge.
 */
class Cross3DPrismEdgeNetwork
{
public:
    using Cell = Cross3D::Cell;
    /*!
     * Build the network of all edges in between horizontally neighboring cells.
     */
    Cross3DPrismEdgeNetwork(const Cross3D& subdivision_structure);
    /*!
     * Get the location of the edge position between two horizontally neighboring cells at a given z.
     */
    Point getCellEdgeLocation(const Cross3D::Cell& before, const Cross3D::Cell& after, const coord_t z) const;
protected:
    const Cross3D* subdivision_structure; //!< The subdivision structure for which the edge network is generated
    std::map<const Cross3D::Cell*, std::vector<Point3>> cell_to_left_edge_locations; //!< The cell edges associated to the more dense cell which is on the right of the edge
    std::map<const Cross3D::Cell*, std::vector<Point3>> cell_to_right_edge_locations; //!< The cell edges associated with the more or equally dense cell which is on the left of the edge

    /*!
     * Make new edges for both sides of the cell if the cell is the owner of the edge on that side.
     * 
     * Add the edge to the mapping.
     */
    void addCellEdges(const Cell& cell);
    /*!
     * Make and add one edge of the cell.
     * The cell is now assumed to be the owner of the edge.
     */
    void addCellEdge(const Cell& cell, std::map<const Cross3D::Cell*, std::vector<Point3>>& cell_to_edge_locations, Cross3D::Direction direction);
    /*!
     * Adjust the end (top or bottom) of an edge to ensure continuity along Z
     */
    void applyOscillationConstraints(const Cell& cell, Cross3D::Direction edge_side, Cross3D::Direction up_down, std::vector<Point3>& edge_locations);
    /*!
     * Adjust the end (top or bottom) of an edge to match a given point
     */
    void adjustEdgeEnd(std::vector<Point3>& edge_locations, Cross3D::Direction up_down, Point3 move_destination);
    // aux:
    char getNeighborDepth(const Cell& cell, Cross3D::Direction direction); //!< Get the depth of the neighboring cell(s) (any will do because neighboring cells cannot have different depths)
};

} // namespace cura


#endif // INFILL_CROSS_3D_PRISM_EDGE_H
