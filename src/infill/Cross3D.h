/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_CROSS_3D_H
#define INFILL_CROSS_3D_H

#include <array>
#include <list>

#include "../utils/optional.h"

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"
#include "../utils/Range.h"
#include "../utils/LineSegment.h"
#include "../utils/SVG.h" // debug

#include "DensityProvider.h"

namespace cura
{

class Cross3DTest; 
/*!
 * 
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
 * |  ↑ \                       |  ↑ \             is_expanding remains   .
 * |  ↑  S\  subdivides into    |S_↑__A\                                  .
 * |  ↑   /                     |S ↑  B/                                  .
 * |  ↑ /                       |  ↑ /             is_expanding remains   .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AS_TO_AB               |B \   AS_TO_BS                           .
 * |    \                       |↖   \             is_expanding remains   .
 * |↖    S\  subdivides into    |S_↖__A\                                  .
 * |  ↖   /                     |S ↑  B/                                  .
 * |    /                       |  ↑ /             is_expanding flips     .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BS               |B \   AS_TO_AB                           .
 * |  ↗ \                       |  ↑ \            is_expanding flips      .
 * |↗    S\  subdivides into    |S_↑__A\                                  .
 * |      /                     |S ↗  B/                                  .
 * |    /                       |↗   /            is_expanding remains    .
 * |A /                         |A /   AS_TO_BS                           .
 * |/                           |/                                        .
 * 
 */
class Cross3D
{
    friend class Cross3DTest;
    using idx_t = int_fast32_t;
public:

    Cross3D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width);

protected:
    static constexpr uint_fast8_t max_subdivision_count = 4; //!< Prisms are subdivided into 2 or 4 prisms
    static constexpr uint_fast8_t number_of_sides = 4; //!< Prisms connect above, below and before and after

    struct Triangle
    {
        /*!
         * The order in
         * Which the edges of the triangle are crossed by the Sierpinski curve.
         */
        enum class Direction
        {
            AC_TO_AB,
            AC_TO_BC,
            AB_TO_BC
        };
        Point straight_corner; //!< C, the 90* corner of the triangle
        Point a; //!< The corner closer to the start of the space filling curve
        Point b; //!< The corner closer to the end of the space filling curve
        Direction dir; //!< The (order in which) edges being crossed by the Sierpinski curve.
        bool straight_corner_is_left; //!< Whether the \ref straight_corner is left of the curve, rather than right. I.e. whether triangle ABC is counter-clockwise

        Triangle(
            Point straight_corner,
            Point a,
            Point b,
            Direction dir,
            bool straight_corner_is_left)
        : straight_corner(straight_corner)
        , a(a)
        , b(b)
        , dir(dir)
        , straight_corner_is_left(straight_corner_is_left)
        {}

        //! initialize with invalid data
        Triangle()
        {}

        std::array<Triangle, 2> subdivide() const;

        //! Get the first edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
        LineSegment getFromEdge() const;
        //! Get the second edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
        LineSegment getToEdge() const;
        //! Get the middle of the triangle
        Point getMiddle() const;
        //! convert into a polyogon with correct winding order
        Polygon toPolygon() const;
    };
    struct Prism
    {
        Triangle triangle;
        Range<coord_t> z_range;
        bool is_expanding; //!< Whether the surface patch to fill this prism has one vertex pointing downward rather than two.

        //! simple constructor
        Prism(
            Triangle triangle,
            coord_t z_min,
            coord_t z_max,
            bool is_expanding
        )
        : triangle(triangle)
        , z_range(z_min, z_max)
        , is_expanding(is_expanding)
        {}

        //! initialize with invalid data
        Prism()
        {}

        bool isHalfCube() const;
        bool isQuarterCube() const;
    };

    enum class Direction : int
    {
        LEFT = 0, // ordered on polarity and dimension: first X from less to more, then Y
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };
    Direction opposite(Direction in);
    uint_fast8_t opposite(uint_fast8_t in);

    struct Cell; // forward decl
    struct Link;
    using LinkIterator = typename std::list<Link>::iterator;
    struct Link
    {
        idx_t to_index;
        std::optional<LinkIterator> reverse; //!< The link in the inverse direction
        float loan; //!< amount of requested_filled_area loaned from one cell to another, when subdivision of the former is prevented by the latter. This value should always be positive.
        idx_t from_index()
        {
            assert(reverse);
            return (*reverse)->to_index;
        }
        Link& getReverse() const
        {
            assert(reverse);
            return **reverse;
        }
        Link(idx_t to_index)
        : to_index(to_index)
        , loan(0.0)
        {}
    };
    struct Cell
    {
        Prism prism;
        idx_t index; //!< index into \ref Cross3D::cell_data
        char depth; //!< recursion depth
        float volume; //!< The volume of the prism in mm^3
        float filled_volume_allowance; //!< The volume to be filled corresponding to the average density requested by the volumetric density specification.
        float minimally_required_density; //!< The largest required density across this area. For when the density specification is the minimal density at each locatoin.
        bool is_subdivided;
        std::array<std::list<Link>, number_of_sides> adjacent_cells; //!< the adjacent cells for each edge/face of this cell. Ordered: before, after, below, above

        std::array<idx_t, max_subdivision_count> children; //!< children. Ordered: down-left, down-right, up-before, up-after

        Cell(const Prism& prism, const idx_t index, const int depth)
        : prism(prism)
        , index(index)
        , depth(depth)
        , volume(-1)
        , filled_volume_allowance(0)
        , minimally_required_density(-1)
        , is_subdivided(false)
        {
//             children.fill(-1); --> done in createTree(...)
        }

        uint_fast8_t getChildCount();
    };

    std::vector<Cell> cell_data; //!< Storage for all the cells. The data of a binary/quaternary tree in depth first order.

    AABB3D aabb;
    int max_depth;
    coord_t line_width; //!< The line width of the fill lines

    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.

    /*!
     * Initialize the the tree structure from the density specification
     */ 
    void initialize();

    float getDensity(const Cell& cell) const;

    Polygon createSierpinski() const;

    std::list<const Cell*> getBottomSequence() const;
private:
    constexpr static uint_fast8_t getNumberOfSides()
    {
        return number_of_sides;
    }

    // Tree creation:
    void createTree();
    void createTree(Cell& sub_tree_root, int max_depth);
    void setVolume(Cell& sub_tree_root);
    void setSpecificationAllowance(Cell& sub_tree_root);

    // Lower bound sequence:
    
    /*!
     * Create a pattern with the required density or more at each location.
     */
    void createMinimalDensityPattern();

    float getActualizedVolume(const Cell& cell) const;
    bool canSubdivide(const Cell& cell) const;
    bool isConstrained(const Cell& cell) const;
    bool isConstrainedBy(const Cell& constrainee, const Cell& constrainer) const;
    
    void subdivide(Cell& cell);
    void initialConnection(Cell& before, Cell& after, Direction dir);
    
    /*!
     * 
     * \param a_to_b The side of \p a to check for being next to cell b. Sides are ordered: before, after, below, above (See \ref Cross3D::Cell::adjacent_cells and \ref Cross3D::Direction)
     */
    bool isNextTo(const Cell& a, const Cell& b, Direction a_to_b) const;

    // output

    void advanceSequence(std::list<const Cell*>& sequence, coord_t new_z) const;

    // debug
    void debugCheckDepths() const;
    void debugCheckVolumeStats() const;

    void debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool horizontal_connections_only) const;
    void debugOutputTriangle(const Triangle& triangle, SVG& svg, float drawing_line_width) const;
    void debugOutputLink(const Link& link, SVG& svg) const;
    void debugOutput(std::list<const Cell*>& sequence, SVG& svg, float drawing_line_width) const;
    void debugOutputTree(SVG& svg, float drawing_line_width) const;
    void debugOutputSequence(SVG& svg, float drawing_line_width) const;
    void debugOutputSequence(const Cell& cell, SVG& svg, float drawing_line_width) const;
};

} // namespace cura


#endif // INFILL_CROSS_3D_H
