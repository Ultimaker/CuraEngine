//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_SIERPINSKI_FILL_H
#define INFILL_SIERPINSKI_FILL_H

#include <list>

#include "../utils/AABB.h"

namespace cura
{

class DensityProvider;
class SierpinskiFillTest;
class SVG;

/*!
 * A class for generating the Cross and Cross 3D infill patterns.
 * 
 * 
 * === BASIC SUBDIVISION ===
 * 
 * The line is generated from a recurvive subdivision of the area of a square into triangles.
 *  _______    _______    _______    _______
 * |      /|  |\     /|  |\  |  /|  |\ /|\ /|          .
 * |    /  |  |  \ /  |  |__\|/__|  |/_\|/_\|  etc     .
 * |  /    |  |  / \  |  |  /|\  |  |\ /|\ /|          .
 * |/______|  |/_____\|  |/__|__\|  |/_\|/_\|          .
 * 
 * Triangles are subdivided into two children like so:
 * |\       |\        .
 * |A \     |A \      .
 * |    \   |    \    . where C is always the 90* straight corner
 * |     C\ |C____B\  .       The direction between A and B is maintained
 * |      / |C    A/
 * |    /   |    /      Note that the polygon direction flips between clockwise and CCW each subdivision
 * |B /     |B /
 * |/       |/
 * 
 * The direction of the space filling curve along each triangle is recorded:
 * 
 * |\                           |\                                        .
 * |B \  AC_TO_BC               |B \   AC_TO_AB                           .
 * |  ↑ \                       |  ↑ \                                    .
 * |  ↑  C\  subdivides into    |C_↑__A\                                  .
 * |  ↑   /                     |C ↑  B/                                  .
 * |  ↑ /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AC_TO_AB               |B \   AC_TO_BC                           .
 * |    \                       |↖   \                                    .
 * |↖    C\  subdivides into    |C_↖__A\                                  .
 * |  ↖   /                     |C ↑  B/                                  .
 * |    /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BC               |B \   AC_TO_AB                           .
 * |  ↗ \                       |  ↑ \                                    .
 * |↗    C\  subdivides into    |C_↑__A\                                  .
 * |      /                     |C ↗  B/                                  .
 * |    /                       |↗   /                                    .
 * |A /                         |A /   AC_TO_BC                           .
 * |/                           |/                                        .
 * 
 * 
 * Each triangle is associated with the length of the Sierpinski dual curve going through the triangle.
 * The realized density of a triangle is calculated using the length of the Sierpinski segment,
 * the line width and the area of the triangle.
 * The realized density is compared with the requested average density over the triangle according to
 * the volumetric density specification of the user.
 * 
 * 
 * 
 * === BALANCING ===
 * 
 * If for any single triangle in the pattern the total realized length of all/both children is less than the average requested length of that parent triangle,
 * the parent triangle would definitely like to be subdivided.
 *     (When dithering, we may subdivide at a lower threshold and a constraint may cause such a triangle not to be subdivided after all. See CONSTRAINTS.)
 * However, it might occur that one child then ends up with a requested length lower than its realized length,
 * namely when the other child has a surplus of requested length.
 * The error thus induced should be recorded.
 * Value will be transported from the child with a surplus to the child with a lack of requested length.
 * 
 * Example:
 * Parent cell realizes 10mm filament, but child 1 requests 100mm and child 2 requests 2mm.
 * Subdivision would lead to two cells with 14mm filament realized.
 * Cell 2 therefore obtains 12mm filament from cell 1, where 12mm is recorded as an error.
 * 
 * === CONSTRAINTS ===
 * 
 * When subdividing an AC_TO_AB or an AB_TO_BC triangle, the place where the Sierpinski curve intersects AB changes.
 * In order to maintain a connected Sierpinski curve, we need to subdivide the (respectively) next / previous as well.
 * A triangle is said to be constrained by a preceding AC_TO_AB triangle or a consecutive AB_TO_BC triangle
 * if the requested density warrants a subdivision, but the connected triangle is of a different recursion depth:
 * _______                                  .
 * \      |\                                .
 *   \⟶⟶⟶⟶|↘ \                              .
 *     \  | ↘/                              .
 *       \|/                                .
 * 
 *         ^^^
 *         constrained triangle
 * ^^^^^^^
 * constraining triangle
 * 
 * 
 * This constraint means that sharp changes in output density are impossible.
 * Instead the output density changes with steps of 1 recursion depth difference at a time.
 * When the input requested density has a sharp contrast, we should try to keep the error introduced by the consecutivity constraint
 * as close to the contrast edge as possible.
 * 
 * 
 * 
 * === TREE VIEW ===
 * 
 * The problem is easiest conceptualized when looking at the tree of all possible triangles.
 * The final sequence of triangles which are crossed by the Sierpinski dual curve can be seen as a cut through the tree.
 *.
 *.                                                  .
 *.                        /\                        .
 *.                      /    \                      .
 *.                    /         \                   .
 *.                  /             \                 .
 *.                /                 \               .
 *.              /                     \             .
 *.            / \                     / \           .
 *.          /     \                 /     \         .
 *.        /         \             /         \       .
 *.      /#\         /#\         / \         / \     .
 *.     /   \       /   \       /    \      /    \   .
 *.   / \   / \   / \   / \   /#\   /#\   / \   / \  .
 *.  /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\/#\/#\/#\/#\ .  The chosen nodes # cut through the tree at a variable depth leading to variable density infill.
 * 
 * 
 * 
 * 
 * === ALGORITHM ===
 * 
 * The algorithm runs as follows:
 * 1- We create the tree of all possible triangles
 * 2- We decide on the lower density boundary sequence of nodes (triangles) to cross
 * 3- We dither by walking over the sequence and deciding whether we should subdivide one level further or not
 *
 * >> 1. Tree creation
 *       *************
 * - Start from the initial boundary and subdivide until the maximum recursion depth
 * - Look up the requested density at leaves and calculate the corresponding line length
 * - Bubble up the total requested line length from the leaves all teh way to the root
 * - Calculate the average length of the Cross Fractal crossing of each triangle and record it
 *
 * 
 * >> 2. Create lower boundary
 *       *********************
 * - Start the sequence of nodes with the starting triangles (We start with two triangles, which form a square)
 * - Iteratively push the cut downward toward the leaves of the tree, while mitigating sharp contrast edges.
 * Each iteration:
 * 1- Move error length values introduced by constraining triangles from constrained triangle to constraining triangle.
 * 2- (Use errors to) subdivide each node where the total errored value is now large enough
 * 
 * 2.1
 * From highest density cells currently in the sequence to lowest density:
 * - Calculate the difference between the actualized length and the requested length.
 * - Distribute this error over the constraining triangles.
 * - Cascade this error value further up toward the root when processing the next density of cells.
 * 
 * 2.2
 * From lowest density cells currently in the sequence to highest density cells:
 * - Try to subdivide the node / two consecutive nodes which share an AB edge
 *   * Use error from 2.1 to subdivide cells which previously could not be subdivided
 *   * Redistribute error to where it came from: cascade errors back down the tree toward teh leaves where the error came from.
 *   * Balance children: make sure each newly introduced child has a positive error value.
 *
 * 
 * >> 3. Dithering
 *       *********
 * From begin to end of the sequence:
 * - If the triangle (or pair of triangles sharing an AB edge) is not constrained:
 *     If the errored value is high enough:
 *       subdivide this/these triangle(s)
 * - Carry along the induced error to the next triangle in the sequence.
 * 
 * 
 * ------------------------------------------------------------------------------------------------------
 * 
 * 
 * By following the path along the middle points of each triangle the Siepinski Curve will be generated.
 * By following the path along the middle of each edge the Cross Fractal curve will be generated, which is the dual of the Sierpinski curve.
 */
class SierpinskiFill
{
    friend class SierpinskiFillTest;
public:
    /*!
     * Basic constructor
     */
    SierpinskiFill(const DensityProvider& density_provider, const AABB aabb, int max_depth, const coord_t line_width, bool dithering);

    ~SierpinskiFill();

    /*!
     * Generate the cross pattern curve
     */
    Polygon generateCross() const; 

    /*!
     * Generate the Sierpinski space filling curve
     */
    Polygon generateCross(coord_t z, coord_t min_dist_to_side, coord_t pocket_size) const; 

    /*!
     * Output the triangles to a canvas
     */
    void debugOutput(SVG& svg);

protected:
    /*!
     * The edge of a triangle.
     */
    struct Edge
    {
        Point l, r; //!< The points left, resp. right of the curve.
    };
    /*!
     * A node in the tree of all triangles.
     * 
     * Vertex C is the straight 90* corner.
     * 
     * Depending on the recursion depth the 90* corner is either on the left or on the right of the curve.
     * 
     * The direction in which this triangle is crossed by the Sierpinski curve is recorded.
     * 
     * Some statistics about the requested and realized polygon length are recorded on each node in the tree.
     */
    struct SierpinskiTriangle
    {
        /*!
         * The order in
         * Which the edges of the triangle are crossed by the Sierpinski curve.
         */
        enum class SierpinskiDirection
        {
            AC_TO_AB,
            AC_TO_BC,
            AB_TO_BC
        };
        Point straight_corner; //!< C, the 90* corner of the triangle
        Point a; //!< The corner closer to the start of the space filling curve
        Point b; //!< The corner closer to the end of the space filling curve
        SierpinskiDirection dir; //!< The (order in which) edges being crossed by the Sierpinski curve.
        bool straight_corner_is_left; //!< Whether the \ref straight_corner is left of the curve, rather than right. I.e. whether triangle ABC is counter-clockwise
        int depth; //!< The recursion depth at which this triangle is generated. Root is zero.

        float area; //!< The area of the triangle in mm^2
        float requested_length; //!< The polyline length corresponding to the average density requested by the volumetric density specification.
        float realized_length; //!< The polyline length of the Cross Fractal line segment which would cross this triangle.
        float total_child_realized_length; //!< The total of the \ref realized_length of all children.
        float error_left; //!< Extra value modulating the \ref requested_length obtained from the triangle on the left / obtained by giving value to the triangle to the left.
        float error_right; //!< Extra value modulating the \ref requested_length obtained from the triangle on the right / obtained by giving value to the triangle to the right.

        SierpinskiTriangle(Point straight_corner, Point a, Point b, SierpinskiDirection dir, bool straight_corner_is_left, int depth)
        : straight_corner(straight_corner)
        , a(a)
        , b(b)
        , dir(dir)
        , straight_corner_is_left(straight_corner_is_left)
        , depth(depth)
        , requested_length(0)
        , total_child_realized_length(0)
        , error_left(0)
        , error_right(0)
        {}
        //! Constructor for the root node.
        SierpinskiTriangle()
        : straight_corner(no_point)
        , a(no_point)
        , b(no_point)
        , depth(0)
        , requested_length(0)
        , total_child_realized_length(0)
        , error_left(0)
        , error_right(0)
        {}
        //! Get the first edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
        Edge getFromEdge();
        //! Get the second edge of this triangle crossed by the Sierpinski and/or Cross Fractal curve.
        Edge getToEdge();
        //! Get the total error value modulating the \ref requested_length
        float getTotalError();
        //! Get the total modulated \ref requested_length
        float getErroredValue();
        //! Get the error induced by subdividing this triangle.
        float getSubdivisionError();
        //! Get the total error currently acting on this traingle.
        float getValueError();
        //! The children into which this triangle would be subdivided. Empty if this is a leaf node.
        std::vector<SierpinskiTriangle> children;
    };


    bool dithering; //!< Whether to oscilate between neighboring realizable density values when the requested density lies in between two possible density values.
    /*!
     * Whether to diffuse errors caused by constraints between consecutive cells.
     * Whether to center a stairway of depths around the contrast edge,
     * rather than keeping the contrast edge on the lower density side only.
     * ==========                 =======___
     * ░░░░░░░░░░===              ░░░░░░░===
     * ░░░░░░░░░░   ===           ░░░░░░░░░░===
     * ░░░░░░░░░░      ======     ░░░░░░░░░░   =========    = : realized value
     * ░░░░░░░░░░░░░░░░░░░░░░     ░░░░░░░░░░░░░░░░░░░░░░    ░ : requested value
     * no constraint error        constraint error
     * diffusion                  diffusion
     */
    bool constraint_error_diffusion;

    //! Whether to use the constraint errors when performing dithering.
    bool use_errors_in_dithering = true;


    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.
    AABB aabb; //!< The square which is the basis of the subdivision of the area on which the curve is based.
    coord_t line_width; //!< The line width of the fill lines
    int max_depth; //!< Maximum recursion depth of the fractal

    SierpinskiTriangle root; //! The (root of the) tree containing all possible triangles of the subdivision.

    /*!
     * The triangles of the subdivision which are crossed by the fractal.
     * This sequence is created by \ref createLowerBoundSequence and updated by \ref diffuseError
     */
    std::list<SierpinskiTriangle*> sequence;


    /*!
     * Calculate all possible subdivision triangles and their statistics.
     * Fill in all fields of each SierpinskiTriangle created, except its \ref error_left and its \ref error_right
     */
    void createTree();

    //! Calculate the direction, orientation and vertices of all nodes in the subtree below this \p sub_root.
    void createTree(SierpinskiTriangle& sub_root);

    //! Calculate the area and realized length of all nodes in the subtree below this \p sub_root.
    void createTreeStatistics(SierpinskiTriangle& sub_root);

    /*!
     * Calculate the requested length of all nodes in the subtree below this \p sub_root from their children.
     * For root nodes, retrieve the requested length from the \ref density_provider.
     */
    void createTreeRequestedLengths(SierpinskiTriangle& sub_root);


    /*!
     * Create the sequence of triangles which have a density just below the requested density,
     * except for where the constraint errors are diffused in order to mitigate sharp contrast edges.
     */
    void createLowerBoundSequence();

    /*!
     * Order the triangles on depth.
     */
    std::vector<std::vector<std::list<SierpinskiTriangle*>::iterator>> getDepthOrdered();

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
     * Subdivide a node into its children.
     * Redistribute leftover errors needed for this subdivision and account for errors needed to keep the children balanced.
     * 
     * \param it iterator to the node to subdivide
     * \param redistribute_errors Whether to redistribute the accumulated errors to neighboring nodes and/or among children
     * \return The last child, so that we can iterate further through the sequence on the input iterator.
     */
    std::list<SierpinskiTriangle*>::iterator subdivide(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool redistribute_errors);


    /*!
     * Redistribute positive errors in as much as they aren't needed to subdivide this node.
     * If this node has received too much positive error then it will subdivide
     * and pass along the error from whence it came.
     * 
     * This is called just before performing a subdivision.
     */
    void redistributeLeftoverErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool distribute_subdivision_errors);

    /*!
     * Balance child values such that they account for the minimum value of their recursion level.
     * 
     * Account for errors caused by unbalanced children.
     * Plain subdivision can lead to one child having a smaller value than the density_value associated with the recursion depth
     * if another child has a high enough value such that the parent value will cause subdivision.
     * 
     * In order to compensate for the error incurred, we more error value from the high child to the low child
     * such that the low child has an erroredValue of at least the density_value associated with the recusion depth.
     * 
     * \param node The parent node of the children to balance
     */
    void balanceErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end);

    /*!
     * Settle down unused errors which have been bubbled up, but haven't been used to subdivide any cell.
     * 
     * Should be called before dithering.
     */
    void settleErrors();

    /*!
     * For each node in the sequence, decide whether we need to subdivide it once more
     * and carry over the induced error to the next node in order to modulate the decision for the next subdivision.
     */
    void diffuseError();

    /*!
     * \return whether a node \p it in the sequence is constrained by the previous node.
     */
    bool isConstrainedBackward(std::list<SierpinskiTriangle*>::iterator it);
    /*!
     * \return whether a node \p it in the sequence is constrained by the next node.
     */
    bool isConstrainedForward(std::list<SierpinskiTriangle*>::iterator it);

    /*!
     * \return the requested value left over if we would subdivide all nodes in the sequence from \p begin to \p end
     */
    float getSubdivisionError(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end);

    /*!
     * Check whether all properties which should hold at any time during the algorithm hold for the current sequence.
     * \param check_subdivision Whether we should check for correct error values / subdivision errors over all nodes.
     */
    void debugCheck(bool check_subdivision = true);
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_H
