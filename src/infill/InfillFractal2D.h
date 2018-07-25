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
 * InfillFractal2D is an abstract class for generating subdivisiong structures with varying levels of subdivision,
 * according to some requested density distribution given by a \ref DensityProvider
 * 
 * This class performs balancing and dithering on a '2D' subdivision fractal,
 * in abstraction of the actual geometric details of the subdivision fractal cells.
 * 
 * The way in which this class is 2D is that the error is dispersed along two directions.
 * For a simple square subdivision grid these are the X and Y axis,
 * but for the Cross3D fractal these are the Z axis and the XY axes.
 * 
 * Cells next to each other are constrained to be at most 1 subdivision level off from each other.
 * 
 * Because the XY plane for the Cross3D fractal is linearized by the Sierpinski-like fractal,
 * the fractal curve can be thought of as a single dimension.
 * A space filling fractal is said to linearize 2D space into a single dimension.
 * 
 * For the sake of simplicity of the explanation, further documentation will assume the terminology square subdivision fractal.
 * 'left' and 'right' would correspond to backward and forward along the space filling curve of a single layer.
 * 
 * For more information on how the 3D space filling surface is generated, see \ref Cross3D
 * 
 * This class deals with cells in abstraction of their geometric shape.
 * The cells are connected to their neighbors above and below, and to the cells left and right.
 * 
 * This class also deals with the tree of possible subdivisions,
 * and actually performs the subdivisions.
 * The connections with neighbors are only recorded for the current actual nodes;
 * i.e. the nodes for which the parents have been chosen to be subdivided.
 * 
 * Several algorithms are implemented to decide the subdivision level at each location.
 * They differ in how constraints between neighbors are resolved and
 * at which side of the requested density a node is - more or less dense than requested.
 * 
 * 
 * 
 * Because a subdivision structure has discrete steps of possible infill density,
 * there is likely a discrepancy between the requested density of a cell and the actualized density.
 * This difference is known as 'quantiziation error'.
 * However, the term 'error' is ambiguous as to the directionality:
 * a negative error could mean there is more requested density than actualized density, but it might as well mean the opposite.
 * 
 * Therefore we adopt the terminology of money: 'value'.
 * The requested amount of material for a cell is called its 'allowance'.
 * If a cell is subdivided more than its allowance then it is said to be 'in debt'.
 * Such a debt can be resolved by taking on a 'loan' from neighboring cells.
 * The value balance of a cell is the allowance plus the total borrowed value minus the value loaned to other cells.
 * 
 * 
 * 
 * The main complexity of this class is in deciding the subdivision level at each location.
 * 
 * 
 * DATA STRUCTURE OVERVIEW
 * =======================
 * All possible cells of the subdivision structure form a tree,
 * with the root node(s) equal to the starting shapes of the subdivision structure.
 * 
 * In order to efficiently store the whole tree, all node are stored in a vector,
 * and instead of having pointers to children on the heap, cells have vector indices as references to their children.
 * Because Cross3D cells sometimes have 2 children and sometimes have 4, the parent child indices cannot be related using a simple mathematical formula.
 * 
 * Next the data structures need to support choosing at which subdivision level the structure will be actualized at each location.
 * Each cell therefore has a field \ref InfillFractal2D::Cell::is_subdivided
 * 
 * Furthermore, neighboring actualized cells are linked to each other in order to efficiently transfer loans between the two.
 * Note that only actualized cells are linked, i.e. cells for which the parent has is_subdivided == true.
 * 
 * For each cardinal direction (left, right, down, up) we link the cells neighboring this cell.
 * Each link has a field to record how much value loan the departure location cell borrows to the destination cell.
 *
 * Each link also has a reference to the iterator to the reverse link.
 * That way we can efficiently insert new links, when changing the link graph due to a subdivision.
 * 
 * The data structure has these two parts at the same time:
 * The tree and the neighborhood graph.
 * 
 * Visualization of double data structure:
 * 
 *                                 root
 *                                  0                          } >> recursion level 0
 * 
 * 
 * 
 *                             2                 3             }
 *                                             ,'|             } >> recursion level 1
 *                    0                   1_,-','|             }
 *                                    ,.-''_,.'/ |                     13, 15, 10 and 11 are linked across different recursion levels to 3; the rest of the links in this figure are within level 2
 *                           ,14----15_,.''   /  |   18    19  }
 *                       _,12----13'-'      /    |16    17     }
 *                   ,.-'   ,.-'           /     |             } >> recursion level 2
 *                ,.6------7-----------,.10------11            }
 *               4------5-'-----------8-------9''              }
 * 
 * Corresponding graph of actualized subdivision structure:
 *
 *            +---+---+-------+
 *            | 14| 15|       |
 *            +---+---+   3   |
 *            | 12| 13|       |
 *            +---+---+---+---+
 *            | 6 | 7 | 10| 11|
 *            +---+---+---+---+
 *            | 4 | 5 | 8 | 9 |
 *            +---+---+---+---+
 * 
 * 
 * 
 * 
 * ALGORITHM OVERVIEW
 * ==================
 * 1) Generate the tree of all possible cells and their subdivisions
 * 2) Decide at which 'height' in the tree the eventual pattern will be: decide on the recursion depth at each location
 * 3) Generating toolpaths
 * 
 * 1) Tree generation
 * - Make (dummy) root node
 * - add first prisms by hand
 * - recursively set the required volume, the actual volume etc.
 * 
 * 2) Create subdivision pattern
 * We start from the root and only decide for each node whether we subdivide it or not;
 * we never 'unsubdivide' a node.
 * All nodes which have been chosen to be subdivided are marked as such in \ref Cross3D::Cell::is_subdivided
 * As such the creation of the subdivision structure is a boundary on the subdivision tree which is only pushed downward toward the leaves of the tree.
 * 
 * The current subdivision structure is a net of linked cells.
 * The cells have the following info:
 * - the geometric prism/square
 * - links to neighboring cells in the current subdivision structure
 * - requested density: allowance
 * 
 * There are several ways in which to decide on the recurison depth at each location.
 * 
 * 2.1) Minimal required density
 * - made easily by recursively subdiviging each cell (and neighbording restructing cells) which is lower than the required density
 * 
 * 2.2) Average density
 * First we decide on the minimal density at each location, while keeping density change cascades balanced around input density requirement changes.
 * Then we apply dithering to get the eventual subdivision pattern
 * 
 * 2.2.1) Average density lower boundary
 * This is 50% of the complexity of this class.
 * This is the most difficult algorithm.
 * Cells with enough allowance to subdivide, but which are constrained by neighbors hand out loans to the contraining neighbor.
 * Near regions where the input density requirement distribution has sharp edges,
 * this causes the change in output density to lie on both sides of the input edge.
 * That way no global error is accumulated because the allowance is invested locally.
 * More on this algorithm below.
 * 
 * 2.2.2) Dithering
 * Walk over the subdivision structure from the left bottom to the top right
 * decide for each cell whether to subdivide once more or not,
 * without reconsidering the thus introduced children for subdivision again.
 * Propagate induced error forward to yet unprocessed cells.
 * 
 * 
 * 3) Generating toolpaths
 * Toolpath generation for square subdivision is quite straight forward. See \ref SquareSubdiv.
 * The following explanation is for \ref Cross3D:
 * For each layer there is a single linear sequence of prisms to cross.
 * In order to efficiently compute the sequence,
 * we compute once a mapping from z coordinates to sequence starting cells
 * and from a starting cell we compute the whole sequence of cells for a given layer.
 * 
 * For such a sequence we look at all triangles of all prisms in the sequence.
 * From this sequence of triangles, we can generate a Sierpinski curve,
 * or the CrossFill curve.
 * When generating the curve, we make sure not to overlap with other line segments in the crossfill pattern.
 * 
 * 
 * 
 * ===========================
 * == Detailed explanations ==
 * ===========================
 * 
 * 
 * 
 * 2.2.1) Average density lower boundary. a.k.a. createBalancedPattern(.)
 * The algorithm for creating the lower bound sequence is as such:
 * do
 *   1. subdivisionPhase(): subdivide each cell which can
 *   2. handOutLoansPhase(): let each cell which wants to subdivide, but which is constrained hand out loans to their constrainers
 * until converged.
 * 
 * It is conjectured that convergence happens at most at step max_depth + 2,
 * meaning that it could suffice to do only max_dpeth + 1 steps, without the extra step to check for convergence.
 * However, I don't have proof.
 * 
 * Both phases handle cells in an order based on cell recursion depth,
 * so that loans or paybacks/settlements cascade.
 * 
 * In the subdivision phase unused loan amounts are paid back just before performing a subdivision.
 * That means we first have to consider the most constraining cell, i.e. the lowest recursion depth cell, closest to the root,
 * so we move from lower recursion depth to higher recursion depth.
 * Then the cells with higher recursion depth (denser cells) get subdivided with the aid of th settlement of previously handled cells.
 * 
 * In the handOutLoansPhase new loans are made;
 * cells which are constrained by neighbors hand out loans to their constrainers.
 * They hand out as much loan as possible given that they would still be able to subdivide;
 * that's how much value they have left over.
 * That means the most constrained cells need to be processed first, i.e. the most dense cells with highest recursion depth.
 * They hand out loans to the constrainers of lower recursion depth, so that they in turn can hand out more loans to their constrainers.
 * 
 * 
 * If there are multiple constrainers or mutiple loans for one given cell in either phase described above,
 * then we need a way to determine which loan will be settled by how much and which constrainer will get which portion of the left-overs.
 * In case of new loans the left over value is divided equally.
 * In case of settlements the loans are paid back proportional to the amount of loan.
 * It is conjectured that these rules lead to allowance being used as locally and efficiently as possible.
 * 
 * 
 * 
 * At the end of the phase (2.2.1) Average density lower boundary / after this phase:
 * all loans handed out are settled back in as much as possible.
 * The loaning should only have effect in phase 2.2.1 and not have any effect on the next phase: dithering.
 * 
 * 
 * -------------------------------------------
 * Subdivision
 * ----
 * When subdividing, we first settle loans with leftover value borrowed - if applicable.  >> settleLoans(.)
 * Then we need to transfer some value in order to have all realized volume accounted for by the allowance.
 * 
 * Performing a subdivision itself induces some value to be transfered, because of two reasons:
 * 1. loans handed out to the parent should be transfered to children
 *      >> transferLoans(.)
 * 2. the children cells are unbalanced, i.e. the allowance of the children differs wildly
 *      >> solveChildDebts(.)
 * 
 * 1. If a cell to be subdivided still had loans from neighboring cells,
 * then these loans need to be transfered to the children.
 * If the children have a deeper recursion depth than the loaner cells,
 * then the loan needs to be split among the children.
 * We divide the loans equally amongst the children - by lack of a principled alternative.
 * 
 * 2. Example: (using two children only, for simplicity)
 * parent has 100 allowance, but realizes 70
 * child_1: has 100 allowance, realizes 50
 * child_2: has 0 allowance, realizes 50
 * (verify that the children together have the same allowance ofs the parent)
 * 
 * child_2 should not be able to exist as such because it doesn't have enough value for its current subdivision level.
 * We therefore need to introduce loans to make each hcild have a non-negative value balance.
 * Because the subdivision is possible we knwo that there is enough vlaue to go around,
 * so we introduce loans between neighboring children until each child has a non-negative balance.
 * 
 * We first apply 1, so that step 2 can take care of any imbalance incurred by either step 1 or by unbalanced children.
 * 
 * 
 * 
 * 
 * -------------------------------------------
 * 2.2.2) Dithering. a.k.a. dither(.)
 * ----
 * 
 * For each cell we determine whether to subdivide it one step further or not,
 * by seeing whether its value balance is more or less than midway between the parent realized density and the children realized density.
 * 
 * The error thus induced is propagated forward to cells yet unprocessed.
 * This error is recorded as a loan - as ususal.
 * 
 * Because the subdivision structure is 2D, the error should be propagated in multiple directions.
 * A simple Floyd-Steinberg dither would work on a grid of same recursion depth cells.
 * The error in FS dithering is propagated to neighboring cells using a partition of unity using the following weights:
 * [ 3 5 1 ]  > cells above
 * [ _ * 7 ]  > forward cell
 *   : :
 *   : ^ current cell
 *   ^ already processed cells
 * 
 * However, because we are dealing with a subdivision structure we cannot process the cells in a row-based manner.
 * We do keep the general row-based order, though:
 *                       +-------+---+---+
 * Recursion rule        |       | 9 | 10| Complex case
 * +---+---+             |   6   +---+---+
 * | 3 | 4 |             |       | 7 | 8 |
 * +---+---+             +---+---+---+---+
 * | 1 | 2 |             | 3 | 4 |       |
 * +---+---+             +---+---+   5   |
 *                       | 1 | 2 |       |
 *                       +---+---+-------+
 * 
 * Note that when processing cells in this order, we cannot propagate error in the usual manner.
 * 1. First we establish that a cell might have multiple neighbors above,
 * as is the case for 5 in the example above: it has 7 and 8 as neighbor above.
 * 
 * 2. Second, some cells cannot propagate error to left-up, because that cell has been processed already,
 * as is the case for 7: above it is 9 and left of it is 6, which is processed before 7.
 * 
 * 3. For some cells, the left up neighbor, or the right up neighbor are equivalent to the upper neighbor,
 * as is the case for 4 and 3 resp.
 * 
 * 
 * In order to solve for a variable amount of neighboring cells,
 * we propagate error based on a partition of unity with weights determined by the direction similar to FS dithering
 * and also influenced by cell size (?)
 * 
 * We don't propagate error to cells already processed by the dithering algorith.
 * Those cells have the flag is_dithered set to true.
 */
template <class CellGeometry>
class InfillFractal2D
{
//     friend class Cross3DTest;
    using idx_t = int_fast32_t;
protected:
    struct Cell; // forward decl
public:


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

protected:
    static constexpr uint_fast8_t max_subdivision_count = 4; //!< Prisms are subdivided into 2 or 4 prisms
    static constexpr uint_fast8_t number_of_sides = 4; //!< Prisms connect above, below and before and after

    /*!
     * Whether the root node is just used to hold the several real first nodes, rather than that the root node corresponds to a geometric cell
     * Differs between square subdivision and cross3D
     */
    bool root_is_bogus;

    InfillFractal2D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width, bool root_is_bogus);

    enum class Direction : int
    {
        LEFT = 0, // ordered on polarity and dimension: first X from less to more, then Y
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };
    Direction opposite(Direction in) const;
    uint_fast8_t opposite(uint_fast8_t in) const;
    uint_fast8_t toInt(Direction in) const { return static_cast<uint_fast8_t>(in); }

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
    /*!
     * get the opposite child side in the X or Z dimension
     * \param dimension 0 for left/right, 1 for up/down
     */
    ChildSide toChildSide(uint_fast8_t in) const;
    uint_fast8_t toInt(ChildSide in) const;
    ChildSide opposite(ChildSide in, uint_fast8_t dimension) const;
    Direction getChildToNeighborChildDirection(ChildSide in, uint_fast8_t dimension) const;

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
        bool is_dithered;
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
        , is_dithered(false)
        {
//             children.fill(-1); --> done in createTree(...)
        }

        uint_fast8_t getChildCount() const;
    };

    std::vector<Cell> cell_data; //!< Storage for all the cells. The data of a binary/quaternary tree in depth first order.

    AABB3D aabb;
    int max_depth;
    coord_t line_width; //!< The line width of the fill lines

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
     * Create the subdivision structure 
     */
    void createBalancedPattern();

    /*!
     * For each node: subdivide if possible and settle loans in as much as they weren't needed to subdivide this node.
     * 
     * Start trying cells with lower recursion level before trying cells with deeper recursion depth, i.e. higher density value.
     * Cascade unused loans downward: from low recursion depth toward the leaves of the subdivision tree.
     * 
     * \return Whether the sequence has changed.
     */
    bool subdivisionPhase();

    /*!
     * Let all constrained nodes hand out loans to their constrainers.
     * 
     * Bubble up errors from nodes which like to subdivide more,
     * but which are constrained by neighboring cells of lower recursion level.
     * 
     * Hand out loans: constrianed cells hand out loans of their unusable error value to the constraining cells,
     * so that the constraining cells can get subdivided, so that the constrained cell is not constrained any more and it can happily subdivide.
     * 
     * Cascade handing out loans upward; from most constrained to constrainers: from leaves toward root.
     * 
     * \return Whether we have redistributed errors which could cause a new subdivision 
     */
    bool handOutLoansPhase();

    /*!
     * Order the cells on depth.
     */
    std::vector<std::vector<Cell*>> getDepthOrdered();
    void getDepthOrdered(Cell& sub_tree_root, std::vector<std::vector<Cell*>>& output);

    void dither(Cell& parent);

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
     * 
     * \return the link from the \p left_right most upstairs neighbor to the diagonal neighbor
     */
    Link* getDiagonalNeighbor(Cell& cell, Direction left_right) const;


    /*!
     * Get the total error currently acting on this cell.
     * Roughly: allowance - realized.
     * 
     * Positive value error means the cell would like to be more dense than it currently is.
     */
    float getValueError(const Cell& cell) const;

    /*!
     * Get the error induced by subdividing this cell.
     * The value error there would be after subdividing this cell.
     * See \ref InfillFractal2D::getValueError.
     */
    float getSubdivisionError(const Cell& node) const;

    /*!
     * Get the total loan error value modulating the \ref requested_length. Used in \ref Cross3D::getSubdivisionError and in \ref Cross3D::getValueError
     * 
     * Positive loan error means other cells loaned this cell more than this cell loaned other cells.
     * 
     * Negative loan errors occur when this cell is loaning its allowance to neighboring cells.
     */
    float getTotalLoanError(const Cell& cell) const;

    /*!
     * Solve debts (negative value error) which were induced by subdivision.
     * 
     * Balance child values such that they account for the minimum value of their recursion level.
     * 
     * Account for errors caused by unbalanced children.
     * Plain subdivision can lead to one child having a smaller value than the density_value associated with the recursion depth
     * if another child has a high enough value such that the parent value will cause subdivision.
     * 
     * In order to compensate for the error incurred, we move error value from the high child to the low child
     * such that the low child has an erroredValue of at least the density_value associated with the recusion depth.
     * 
     * \param parent The parent node of the children to balance
     */
    void solveChildDebts(const Cell& parent);

    /*!
     * Transfer the loans from an old link to the new links after subdivision
     */
    void transferLoans(Link& old, const std::list<Link*>& new_links);

    /*!
     * Pay back loans.
     * 
     * Redistribute positive errors in as much as they aren't needed to subdivide this node.
     * If this node has received too much positive error value then it will subdivide
     * and pass along the error from whence it came.
     * 
     * This is called just before performing a subdivision.
     * It is also called after createBalancedPattern on all nodes, see \ref InfillFractal2D::settleLoans.
     */
    void settleLoans(Cell& from, float left_overs);

    /*!
     * Settle loans handed out during createBalancedPattern.
     * 
     * Redistribute leftover unused error value:
     * cascade downward to constrained cells.
     * 
     * Similar to \ref SierpinskiFill::settleErrors
     */
    void settleLoans();

    /*!
     * Transfer value from one cell to the other.
     * 
     * If the other cell already transfered value to the sender, reduce that loan first.
     */
    void transferValue(Link& transfer_direction, float loan_value);

    /*!
     * Get the total actualized volume of the current subdivision structure.
     * Add all actualized volume of each non-subdivvided node together.
     */
    float getTotalActualizedVolume(const Cell& sub_tree_root);

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
    void debugCheckLoans(const Cell& cell) const;

    virtual void debugOutput(SVG& svg, float drawing_line_width, bool draw_arrows) const = 0;
};

} // namespace cura

#include "InfillFractal2D.hpp"

#endif // INFILL_INFILL_FRACTAL_2D_H
