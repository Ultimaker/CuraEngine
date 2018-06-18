/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_INFILL_FRACTAL_2D_H
#define INFILL_INFILL_FRACTAL_2D_H

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

/*!
 * InfillFractal2D is a class for generating the Cross 3D infill pattern with varying density accross X, Y and Z.
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
 * This class effectively combines the algorithms from \ref SierpinskiFill and \ref SquareSubdivision.
 * Because the space filling curve can be seen as a way to linearize 2D space,
 * the 3D space filling surface is similar to a 2D square subdivision grid.
 * Quantization error is the error between actual density of a prism cell and required density of the user specified distribution.
 * Quantizatoin error should be distributed in two dimensions:
 * 'left' and 'right' that is: along the curve itself in the XY plane
 * up and down in the Z dimension.
 * 
 * 
 * We start with a cubic aabb and splice that vertically into 2 prisms of type I: half-cubic.
 * A half-cubic prism is subdivided vertically into 2 quarter-cubic prisms.
 * A quarter cubic prism is subdivided vertically and horizontally into 4 half-cubic prisms.
 * 
 * 
 * 
 * ALGORITHM OVERVIEW
 * ==================
 * 1) Generate the tree of all prisms and their subdivisions
 * 2) Decide at which 'height' in the tree the eventual pattern will be: decide on the recursion depth at each location
 * 3) Walk from the bottom to the top and create layers
 * 
 * 1) Tree generation
 * - Make dummy root node
 * - add first prisms by hand
 * - recursively set the sierpinski connectivity of each cell, the required volume, the actual volume etc.
 * 
 * 2) Create subdivision pattern
 * We start from the root and only decide for each node whether we subdivide it or not;
 * we never 'unsubdivide' a node.
 * All nodes which have been chosen to be subdivided are marked as such in \ref Cross3D::Cell::is_subdivided
 * As such the creation of the subdivision structure is a boundary on the subdivision tree which is only pushed downward toward the leaves of the tree.
 * 
 * The current subdivision structure is a net of linked cells.
 * The cells have the following info:
 * - the geometric prism
 * - links to neighboring cells in the current subdivision structure
 * - error values of built up and redistributed quatization error
 * 
 * There are several ways in which to decide on the recurison depth at each location.
 * 
 * 2.1) Minimal required density
 * - made easily by recursively subdiviging each cell (and neighbording restructing cells) which is lower than the required density
 * 
 * 2.2) Average density
 * First we decide on the minimal density at each location, while keeping density change cancades balanced around input density requirement changes.
 * Then we apply dithering to get the eventual subdivision pattern
 * 
 * 2.2.1) Average density lower boundary
 * This is 50% of the complexity of this class.
 * This is the most difficult algorithm.
 * Induced quantization errors are redistributed to nearest cells, so as to make the subdivision structure balanced.
 * If no errors would have been distributed the final pattern would either be too dense or too sparse
 * near regions where the input density requirement distribution has sharp edges.
 * Such errors cause problems for the next dithering phase, which would then oscillate between several subdivision levels too dense and several subdivision levels too sparse
 * in the regions just after the sharp density edges.
 * 
 * 2.2.2) Dithering
 * Walk over the subdivision struction from the left bottom to the top right
 * decide for each cell whether to subdivide once more or not,
 * without reconsidering the thus introduced children for subdivision again.
 * 
 * 
 * 3) Walking across layers
 * For each layer there is a single linear sequence of prisms to cross.
 * In order to efficiently compute the sequence,
 * we compute the bottom sequence once
 * and update it for each layer when it crosses the top of any prism in the sequence.
 * 
 * For such a sequence we look at all triangles of all prisms in the sequence.
 * From this sequence of triangles, we can generate a Sierpinski curve,
 * or the CrossFill curve.
 * When generating the curve, we make sure not to overlap with other line segments in the crossfill pattern.
 * 
 * 
 * 
 */
template <class CellGeometry>
class InfillFractal2D
{
//     friend class Cross3DTest;
    using idx_t = int_fast32_t;
protected:
    struct Cell; // forward decl
public:

    InfillFractal2D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width);

    virtual ~InfillFractal2D();
    
    /*!
     * Initialize the the tree structure from the density specification
     */ 
    void initialize();

    /*!
     * Create a pattern with the required density or more at each location.
     */
    void createMinimalDensityPattern();

    /*!
     * TODO
     */
    void createDitheredPattern();

    /*!
     * Create a pattern with no dithering and no balancing.
     * 
     * \param middle_decision_boundary Whether to divide when required volume is more than midway between the actualized volume and the children actualized volume. Set to false to create the minimal required subdivision before dithering
     */
    void createMinimalErrorPattern(bool middle_decision_boundary = true);

    /*!
     * Create the subdivision structure 
     */
    void createBalancedPattern();

protected:
    static constexpr uint_fast8_t max_subdivision_count = 4; //!< Prisms are subdivided into 2 or 4 prisms
    static constexpr uint_fast8_t number_of_sides = 4; //!< Prisms connect above, below and before and after

    enum class Direction : int
    {
        LEFT = 0, // ordered on polarity and dimension: first X from less to more, then Y
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };

    /*!
     * The direction from the middle of the parent prism to a child prism
     */
    enum class ChildSide : int
    {
        LEFT_BOTTOM = 0,
        RIGHT_BOTTOM = 1,
        LEFT_TOP = 2,
        RIGHT_TOP = 3,
        COUNT = 4,
        FIRST = LEFT_BOTTOM
    };
    Direction opposite(Direction in);
    uint_fast8_t opposite(uint_fast8_t in);

    struct Link; // fwd decl
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
//     template<typename CellGeometry>
    struct Cell
    {
        CellGeometry elem;
        idx_t index; //!< index into \ref Cross3D::cell_data
        char depth; //!< recursion depth
        float volume; //!< The volume of the prism in mm^3
        float filled_volume_allowance; //!< The volume to be filled corresponding to the average density requested by the volumetric density specification.
        float minimally_required_density; //!< The largest required density across this area. For when the density specification is the minimal density at each locatoin.
        bool is_subdivided;
        std::array<std::list<Link>, number_of_sides> adjacent_cells; //!< the adjacent cells for each edge/face of this cell. Ordered: before, after, below, above

        std::array<idx_t, max_subdivision_count> children; //!< children. Ordered: down-left, down-right, up-left, up-right

        Cell(const CellGeometry& elem, const idx_t index, const int depth)
        : elem(elem)
        , index(index)
        , depth(depth)
        , volume(-1)
        , filled_volume_allowance(0)
        , minimally_required_density(-1)
        , is_subdivided(false)
        {
//             children.fill(-1); --> done in createTree(...)
        }

        uint_fast8_t getChildCount() const;
    };

    std::vector<Cell> cell_data; //!< Storage for all the cells. The data of a binary/quaternary tree in depth first order.

    AABB3D aabb;
    int max_depth;
    coord_t line_width; //!< The line width of the fill lines
    coord_t min_dist_to_cell_bound; //!< The minimal distance between a triangle vertex and the space filling surface oscillating vertex for a straight corner or straight edge
    coord_t min_dist_to_cell_bound_diag; //!< The minimal diagonal distance between a triangle vertex and the space filling surface oscillating vertex

    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.


    virtual float getDensity(const Cell& cell) const = 0;

protected:
    static constexpr float allowed_volume_error = .0001; // a 10th of a cubic 0.1mm 
    constexpr static uint_fast8_t getNumberOfSides()
    {
        return number_of_sides;
    }

    // Tree creation:
    virtual void createTree() = 0;
    virtual void setVolume(Cell& sub_tree_root) = 0;
    void setSpecificationAllowance(Cell& sub_tree_root);

    // Lower bound sequence:

    /*!
     * For each noe: subdivide if possible.
     * 
     * Start trying cells with lower recursion level before trying cells with deeper recursion depth, i.e. higher density value.
     * 
     * \return Whether the sequence has changed.
     */
    bool subdivideAll();

    /*!
     * Bubble up errors from nodes which like to subdivide more,
     * but which are constrained by neighboring cells of lower recursion level.
     * 
     * \return Whether we have redistributed errors which could cause a new subdivision 
     */
    bool bubbleUpConstraintErrors();

    /*!
     * Order the triangles on depth.
     */
    std::vector<std::vector<Cell*>> getDepthOrdered();
    void getDepthOrdered(Cell& sub_tree_root, std::vector<std::vector<Cell*>>& output);

    void dither(Cell& parent, std::vector<ChildSide>& tree_path);

    virtual float getActualizedVolume(const Cell& cell) const = 0;
    virtual float getChildrenActualizedVolume(const Cell& cell) const;
    virtual bool canSubdivide(const Cell& cell) const;
    virtual bool isConstrained(const Cell& cell) const;
    virtual bool isConstrainedBy(const Cell& constrainee, const Cell& constrainer) const;

    /*!
     * Subdivide a node into its children.
     * Redistribute leftover errors needed for this subdivision and account for errors needed to keep the children balanced.
     * 
     * \param cell the cell to subdivide
     * \param redistribute_errors Whether to redistribute the accumulated errors to neighboring nodes and/or among children
     */
    void subdivide(Cell& cell, bool redistribute_errors);
    void initialConnection(Cell& before, Cell& after, Direction dir);
    
    /*!
     * 
     * \param a_to_b The side of \p a to check for being next to cell b. Sides are ordered: before, after, below, above (See \ref Cross3D::Cell::adjacent_cells and \ref Cross3D::Direction)
     */
    virtual bool isNextTo(const Cell& a, const Cell& b, Direction a_to_b) const = 0;

    /*!
     * Get the right up diagonal neighbor
     * for which the left lower corner coincides with the right uper corner of this cell
     * if any, otherwise return nullptr
     *           __ __
     * :        |__|__|             but dont get this one  or this one
     * :________|▓▓|__|                           ^             ^
     * |       ↗|     |             | X X X X X |         |     |  X
     * | from   |_____|             |___________|         |_____|  X
     * |        |     |             |from |     |         |from |  X
     * |________|_____|             |_____|_____|         |_____|____
     */
    Link* getDiagonalNeighbor(Cell& cell, Direction left_right) const;


    /*!
     * Check whetehr we can propagate error to the cell diagonally left up of this cell.
     * This is only possible of we hadn't already processed that cell,
     * which is the case if this is a ll cell after a lb cell after the last right cell.
     */
    bool canPropagateLU(Cell& cell, const std::vector<ChildSide>& tree_path);

    //! Get the error induced by subdividing this cell.
    float getSubdivisionError(const Cell& node) const;
    //! Get the total error currently acting on this traingle.
    float getValueError(const Cell& cell) const;
    //! Get the total loan error value modulating the \ref requested_length. Used in \ref Cross3D::getSubdivisionError and in \ref Cross3D::getValueError
    float getTotalLoanError(const Cell& cell) const;
    //! Get the total loaned amount which neighbors have loaned to this cell
    float getTotalLoanObtained(const Cell& cell) const;

    /*!
     * Transfer the loans from an old link to the new links after subdivision
     */
    void transferLoans(Link& old, const std::list<Link*>& new_links);

    void distributeLeftOvers(Cell& from, float left_overs);

    /*!
     * Subdivide cells once more if it doesn't matter for the density but it does matter for the oscillation pattern.
     * Subdivide AC_TO_BC quarter-cubes if neighboring cells are subdivided more.
     */
    void sanitize(Cell& sub_tree_root);

    //----------------------
    //------ OUTPUT --------
    //----------------------

    // debug
    virtual void debugCheckChildrenOverlap(const Cell& cell) const = 0;
    void debugCheckDepths() const;
    void debugCheckVolumeStats() const;
};

} // namespace cura

#include "InfillFractal2D.hpp"

#endif // INFILL_INFILL_FRACTAL_2D_H
