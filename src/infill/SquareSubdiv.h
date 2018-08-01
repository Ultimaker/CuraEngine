/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_SQUARE_SUBDIV_H
#define INFILL_SQUARE_SUBDIV_H

#include <array>
#include <list>

#include "../utils/optional.h"

#include "../utils/IntPoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"
#include "../utils/Range.h"
#include "../utils/LineSegment.h"
#include "../utils/SVG.h" // debug

#include "DensityProvider.h"

#include "InfillFractal2D.h"

namespace cura
{

class SquareSubdivTest; // to be friends with

/*!
 * A square subdivision structure to generate a grid infill pattern with varying infill density,
 * while maintaining connectivity of infill lines
 * and for creating a Hilbert curve with varying density.
 * 
 * This class inherits from InfillFractal2D, so it can use an arbitrary \ref DensityProvider,
 * and it implements several ways in which to adhere to such a density-provider.
 * 
 * The subdivision fractal starts with a single axis aligned bounding box,
 * and each subdivision step divides a square into 4 smaller ones.
 * 
 * A Moore curve is the same as a Hilbert curve, except for the initial subdivision step.
 * That step makes it so that in any following step the fractal is a single closed polygon.
 * 
 * 
 * 
 * Hilbert (+underlying square subdivision structure)                (without underlying subdiv)
 * +---------------+     +-------+-------+      +---+---+---+---+      _   _   _   _
 * |               |     |       |       |      | o---o | o---o |     | |_| | | |_| |
 * |               |     |   o-------o   |      +-|-+-|-+-|-+-|-+     |_   _| |_   _|
 * |               |     |   |   |   |   |      | o | o---o | o |      _| |_____| |_ 
 * |       o........     +---|---+---|---+      +-|-+---+---+-|-+     |  ___   ___  |
 * |       :       |     |   |   |   |   |      | o---o | o---o |     |_|  _| |_  |_|
 * |       :       |     |   o   |   o....      +---+-|-+-|-+---+      _  |_   _|  _
 * |       :       |     |   :   |       |      | o---o | o---o..     | |___| |___| |
 * +-------:-------+     +---:---+-------+      +-:-+---+---+---+
 * 
 * Moore (+underlying square subdivision structure)                  (without underlying subdiv)
 * +---------------+     +-------+-------+      +---+---+---+---+      _   _   _   _
 * |               |     |       |       |      | o---o | o---o |     | |_| | | |_| |
 * |               |     |   o-------o   |      +-|-+-|-+-|-+-|-+     |_   _| |_   _|
 * |               |     |   |   |   |   |      | o | o---o | o |      _| |_____| |_
 * |       o       |     +---|---+---|---+      +-|-+---+---+-|-+     |_   _____   _|
 * |               |     |   |   |   |   |      | o | o---o | o |      _| |_   _| |_
 * |               |     |   o-------o   |      +-|-+-|-+-|-+-|-+     |  _  | |  _  |
 * |               |     |       |       |      | o---o | o---o |     |_| |_| |_| |_|
 * +---------------+     +-------+-------+      +---+---+---+---+
 * 
 * 
 */
class SquareSubdiv : public InfillFractal2D<AABB>
{
    friend class SquareSubdivTest;
    using idx_t = int_fast32_t;
    using Base = InfillFractal2D<AABB>;
public:
    using Cell = Base::Cell;

    SquareSubdiv(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width, bool space_filling_curve);

    Polygon createMooreLine() const;

    void debugOutput(SVG& svg, float drawing_line_width, bool draw_arrows) const;
protected:

    float getDensity(const Cell& cell, const int_fast8_t averaging_statistic) const;

private:
    bool space_filling_curve; // Whether this instance is used to compute the space filling curve rather than the squares. 
    constexpr static uint_fast8_t getNumberOfSides()
    {
        return number_of_sides;
    }

    // Tree creation:
    void createTree();
    void createTree(Cell& sub_tree_root);
    //! Create the tree for a first child of root
    void setVolume(Cell& sub_tree_root);

    // Lower bound sequence:

    float getActualizedVolume(const Cell& cell) const;

    /*!
     * 
     * \param a_to_b The side of \p a to check for being next to cell b. Sides are ordered: before, after, below, above (See \ref Cross3D::Cell::adjacent_cells and \ref Cross3D::Direction)
     */
    bool isNextTo(const Cell& a, const Cell& b, Direction a_to_b) const;

    //----------------------
    //------ OUTPUT --------
    //----------------------

    std::vector<const Cell*> createMoorePattern() const;
    
    void createMoorePattern(const Cell& sub_tree_root, std::vector<const Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir) const;
    void createHilbertPattern(const Cell& sub_tree_root, std::vector<const Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir) const;

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

    //----------------------
    //------ DEBUG ---------
    //----------------------
    void debugCheckChildrenOverlap(const Cell& cell) const;

    void debugOutput(SVG& svg, const Cell& sub_tree_root, float drawing_line_width, bool draw_arrows) const;
    void debugOutputLink(const Link& link, SVG& svg) const;
    void debugOutputTree(SVG& svg, float drawing_line_width) const;
    void debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool horizontal_connections_only) const;
    void debugOutputSquare(const AABB& square, SVG& svg, float drawing_line_width) const;
    void debugOutputSequence(SVG& svg, float drawing_line_width) const;
    void debugOutputSequence(const Cell& cell, SVG& svg, float drawing_line_width) const;
};

} // namespace cura


#endif // INFILL_SQUARE_SUBDIV_H
