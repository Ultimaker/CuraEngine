/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_CROSS_3D_H
#define INFILL_CROSS_3D_H

#include <array>
#include <list>
#include <map>

#include "../utils/optional.h"

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"
#include "../utils/Range.h"
#include "../utils/LineSegment.h"
#include "../utils/SVG.h" // debug

#include "DensityProvider.h"

#include "InfillFractal2D.h"
#include "Cross3DPrism.h"

namespace cura
{

class Cross3DTest; // fwd decl
/*!
 * Cross3D is a class for generating the Cross 3D infill pattern with varying density accross X, Y and Z.
 * Each layer is a space filling curve and across the layers the pattern forms a space filling surface,
 * which satisfied overhang angle and has no bridges.
 * 
 * The 3D surface is oscillating / pulsating in and out across the layers.
 * This way it touches itself and creates a foam like structure which is similarly flexible in all directions.
 * 
 * An underlying subdivision structure is generated, which subdivides the 3D space into regular parts: prisms.
 * These prisms have a triangular base and rectangular sides.
 * The prism heights correspond to half the wave length of the oscillation pattern:
 * the surface patch across a prism is either expanding or contracting.
 * 
 * For a layer each prism crossing that layer is sliced into a triangle.
 * The triangle grid thus created for a layer is then used to generate a Sierpinski-like fractal, similar to \ref SierpinskiFill
 * 
 * TODO: remove the code duplication of the SierpinskiTriangle.
 * 
 * Because the space filling curve can be seen as a way to linearize 2D space,
 * the 3D space filling surface is similar to a 2D square subdivision grid.
 * Quantization error is the error between actual density of a prism cell and required density of the user specified distribution.
 * Quantizatoin error should be distributed in two dimensions:
 * 'left' and 'right' that is: along the curve itself in the XY plane
 * up and down in the Z dimension.
 * 
 * This class inherits from InfillFractal2D so that it can apply the 2D dithering principles to distribute
 * 
 * 
 * We start with a cubic aabb and splice that vertically into 2 prisms of type I: half-cubic.
 * A half-cubic prism is subdivided vertically into 2 quarter-cubic prisms.
 * A quarter cubic prism is subdivided vertically and horizontally into 4 half-cubic prisms.
 * 
 * 
 * 
 * This class is to be used as follows:
 * 1- Create an instance
 * 2- call some functions to initialize and to decide the subdivision level at each location
 * 3- on each layer: call function to generate a path from the subdivision structure
 * 
 * The most difficult algorithms cover deciding the subdivision level at each location;
 * they are handled with in \ref InfillFractal2D.
 * See the class documentation of that class to get a better picture of how Cross3D infill works.
 * 
 * The most difficulty in this class is in converting a cell to the segment of the space filling curve.
 * There is quite some functionality in place to circumvent places in the cell which could lead to overlapping toolpaths.
 * Also realizing pockets in the pattern is rather difficult if neighboring cells differ in subdivision level.
 * 
 * 
 * 
 * Triangle subdivision explanation adopted from SierpinskiFill
 * ------------------------------------------------------------
 * Triangles are subdivided into two children like so:
 * |\       |\        .
 * |A \     |A \      .
 * |    \   |    \    . where S is always the 90* straight corner
 * |     S\ |S____B\  .       The direction between A and B is maintained
 * |      / |S    A/
 * |    /   |    /      Note that the polygon direction flips between clockwise and SSW each subdivision
 * |B /     |B /
 * |/       |/
 * 
 * The direction of the space filling curve along each triangle is recorded:
 * 
 * |\                           |\                                        .
 * |B \  AS_TO_BS               |B \   AS_TO_AB                           .
 * |  ↑ \                       |  ↑ \                                    .
 * |  ↑  S\  subdivides into    |S_↑__A\                                  .
 * |  ↑   /                     |S ↑  B/                                  .
 * |  ↑ /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AS_TO_AB               |B \   AS_TO_BS                           .
 * |    \                       |↖   \                                    .
 * |↖    S\  subdivides into    |S_↖__A\                                  .
 * |  ↖   /                     |S ↑  B/                                  .
 * |    /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BS               |B \   AS_TO_AB                           .
 * |  ↗ \                       |  ↑ \                                    .
 * |↗    S\  subdivides into    |S_↑__A\                                  .
 * |      /                     |S ↗  B/                                  .
 * |    /                       |↗   /                                    .
 * |A /                         |A /   AS_TO_BS                           .
 * |/                           |/                                        .
 * 
 */
class Cross3D : public InfillFractal2D<Cross3DPrism>
{
    friend class Cross3DTest;
    using idx_t = int_fast32_t;
public:
    using Cell = InfillFractal2D<Cross3DPrism>::Cell;
    using Triangle = Cross3DPrism::Triangle;
    using Prism = Cross3DPrism;

    Cross3D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width);

    /*!
     * Simple wrapper class for the structure which walks through slices of the subdivision structure
     */
    struct SliceWalker
    {
        std::list<const Cell*> layer_sequence; //!< The sequence of cells on a given slice through the subdivision structure. These are in Sierpinski order.
    };

    /*!
     * Subdivide cells once more if it doesn't matter for the density but it does matter for the oscillation pattern.
     * Subdivide AC_TO_BC quarter-cubes if neighboring cells are subdivided more.
     */
    void sanitize();

    /*!
     * Get a list of sequence start cells for each layer.
     * 
     * Create a mapping from the bottom z coord of each cell to the cell for each left-most cell along the Z axis.
     * 
     * This is a std::map so that we can lookup the cell from from the z height of a layer (rather than the exact z height of the cell).
     * 
     * \note Note that map.lower_bound and map.upper_bound both always return an iterator pointing to the the element with a _higher_ key (or equal).
     */
    std::map<coord_t, const Cell*> getSequenceStarts() const;

    SliceWalker getSequence(const Cell& start_cell, coord_t z) const;

    Polygon generateSierpinski(const SliceWalker& sequence) const;

    Polygon generateCross(const SliceWalker& sequence, coord_t z) const;
protected:
    coord_t min_dist_to_cell_bound; //!< The minimal distance between a triangle vertex and the space filling surface oscillating vertex for a straight corner or straight edge
    coord_t min_dist_to_cell_bound_diag; //!< The minimal diagonal distance between a triangle vertex and the space filling surface oscillating vertex

    float getDensity(const Cell& cell, const int_fast8_t averaging_statistic) const;

private:
    constexpr static uint_fast8_t getNumberOfSides()
    {
        return number_of_sides;
    }

    // Tree creation:
    void createTree();
    //! Create the tree for a first child of root
    void createTree(const Triangle& triangle, size_t root_child_number);
    void createTree(Cell& sub_tree_root, int max_depth);
    void setVolume(Cell& sub_tree_root);

    // Lower bound sequence:

    float getActualizedVolume(const Cell& cell) const;

    /*!
     * 
     * \param a_to_b The side of \p a to check for being next to cell b. Sides are ordered: before, after, below, above (See \ref Cross3D::Cell::adjacent_cells and \ref Cross3D::Direction)
     */
    bool isNextTo(const Cell& a, const Cell& b, Direction a_to_b) const;

    /*!
     * Subdivide cells once more if it doesn't matter for the density but it does matter for the oscillation pattern.
     * Subdivide AC_TO_BC quarter-cubes if neighboring cells are subdivided more.
     */
    void sanitize(Cell& sub_tree_root);

    //----------------------
    //------ OUTPUT --------
    //----------------------

    /*!
     * Get a list of sequence start cells for each layer for everyhting falling under \p sub_tree_root.
     * 
     * Create a mapping from the bottom z coord of each cell to the cell for each left-most cell along the Z axis.
     * 
     * This is a std::map so that we can lookup the cell from from the z height of a layer (rather than the exact z height of the cell).
     * 
     * \note Note that map.lower_bound and map.upper_bound both always return an iterator pointing to the the element with a _higher_ key (or equal).
     * 
     * \param sub_tree_root The node in the tree under which to get the leftmost leaves
     * \param[out] output Where to store the leftmost leaves
     */
    void getSequenceStarts(const Cell& sub_tree_root, std::map<coord_t, const Cell*>& output) const;

    /*!
     * Add line segments for the space filling surface patch in this \p cell.
     * This depends on neighboring cells and their upstairs and downstrairs neighbors.
     * 
     * \param cell The cell to slice
     * \param after The cell after/right of the cell to slice at the slicing height
     * \param z The height at which to slice the cell
     * \param[in,out] from The to-location of the previous cell (input) and the from-location of the next cell (output)
     * \param[out] output Where to add the points
     */
    void sliceCell(const Cell& cell, const Cell& after, const coord_t z, Point& from, PolygonRef output) const;

    /*!
     * Get the location of a vertex of the space filling curve lying on the edge in between two cells.
     * This function applies XY neighboring pattern constraints (highest recursion dpeth decides oscillation pattern)
     * and Z oscillation continuity constraints (oscillation pattern is altered to fit higher recursion cells above and below).
     * 
     * Also the points are generated not too close to the cell boundary so as not to cause line overlap problems.
     */
    Point getCellEdgeLocation(const Cell& before, const Cell& after, const coord_t z) const;

    /*!
     * Change the vertex position of the space filling curve along an edge
     * in order to make the oscillation pattern fit with more dense cells either above or below.
     * 
     * The input and output vertex position is/should not be corrected to not lie too close to the borders yet.
     * 
     * \param before The cell left of the edge at this height
     * \param after The cell right of the edge at this height
     * \param z The z height at which we are slicing the cell
     * \param densest_cell Either \p before or \p after; whichever is deeper recursed
     * \param edge The edge of the \p densest_cell; i.e. the shortest edge in common to both \p before and \p after.
     * \param edge_size Precomputed length of \p edge
     * \param checking_direction Either UP or DOWN, to specify in which direction to check for constraints
     * \param[in,out] pos The position along the edge to be altered by this function
     */
    void applyZOscillationConstraint(const Cell& before, const Cell& after, coord_t z, const Cell& densest_cell, const LineSegment edge, const coord_t edge_size, const Direction checking_direction, coord_t& pos) const;

    /*!
     * Get the position in the oscillation pattern based on the two cells on both sides of the edge,
     * without taking the edges above and below into account.
     */
    coord_t getCellEdgePosition(const Cell& cell, const coord_t edge_size, coord_t z) const;

    /*!
     * Whether the space filling curve \p segment is overlapping beyond the \p edge
     * Whether the angle between the \p edge and the \p segment is < 45*
     * The from point of \p segment is assumed to lie on the \p edge
     */
    bool isOverlapping(const LineSegment edge, const LineSegment segment) const;

    //! Add a 45 degree bend in order to avoid line overlap in the curve patch along a triangle
    void add45degBend(const Point end_point, const Point other_end_point, const LineSegment edge, PolygonRef output) const;

    //----------------------
    //------ DEBUG ---------
    //----------------------

    void debugCheckHeights(const SliceWalker& sequence, coord_t z) const;
    void debugCheckChildrenOverlap(const Cell& cell) const;

    void debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool draw_arrows, bool horizontal_connections_only) const;
    void debugOutputTriangle(const Triangle& triangle, SVG& svg, float drawing_line_width) const;
    void debugOutputLink(const Link& link, SVG& svg) const;
    void debugOutput(const SliceWalker& sequence, SVG& svg, float drawing_line_width) const;
    void debugOutputTree(SVG& svg, float drawing_line_width) const;
    void debugOutput(SVG& svg, float drawing_line_width, bool draw_arrows) const;
    void debugOutputSequence(const Cell& cell, SVG& svg, float drawing_line_width, bool draw_arrows) const;
};

} // namespace cura


#endif // INFILL_CROSS_3D_H
