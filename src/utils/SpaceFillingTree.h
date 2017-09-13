/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_SPACE_FILLING_TREE_H
#define UTILS_SPACE_FILLING_TREE_H

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/intpoint.h"
#include "../utils/optional.h"

#include "../utils/SVG.h"

namespace cura
{

class SpaceFillingTreeFill;
/*!
 *                 |
 *               --+--
 *            |    |    |                              ▄▉▄
 *          --+----+----+--                          ▄  ▉  ▄
 *       |    |    |    |                           ▀▉▀▀▉▀▀▉▀
 *     --+--     --+--     --+--                 ▄▉▄   ▄▉▄   ▄▉▄
 *  |    |    |    |    |    |    |            ▄  ▉  ▄  ▉  ▄  ▉  ▄
 *--+----+----+----+----+----+----+--         ▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀
 *  |    |    |    |    |    |    |              ▄▉▄   ▄▉▄   ▄▉▄
 *     --+--     --+--     --+--                  ▀  ▄  ▉  ▄  ▀
 *       |    |    |    |                           ▀▉▀▀▉▀▀▉▀
 *          --+----+----+--                            ▄▉▄
 *            |    |    |                               ▀
 *               --+--
 *                 |
 *
 * Class for generating a space filling tree which consists of a cros fractal.
 * The limit of the fractal sovers the area of a square.
 * 
 * The initial line segments have half the width of the full square;
 * The next iteration adds line segments of half that width to the end;
 * because 1+.5+.25+... = 2 this eventually fills the square
 * 
 * The root of the tree is the middle.
 * 
 * The connected distance from a point to the root determines it's parent-child relation;
 * no child is along the edge from a node to it's parent.
 */
class SpaceFillingTree
{
public:
    class Node;
    /*!
     * pure virtual class to implement by an external class
     * which is used to process nodes when doing a DFS walk.
     */
    class LocationVisitor
    {
    public:
        /*!
         * Register a location being crossed during the walk.
         * \param node The node visited
         */
        virtual void visit(const Node* node) = 0;
    };

    /*!
     * Construct a tree
     * \param middle The location of the root
     * \param radius The distance from the root to the side (or the top etc.)
     * \param depth The recursion depth of the fractal
     */
    SpaceFillingTree(
        Point middle,
        coord_t radius,
        unsigned int depth);

    /*!
     * Move constructor
     */
    SpaceFillingTree(SpaceFillingTree&& other);

    /*!
     * Destructor, which recursively deletes all nodes in the tree.
     */
    ~SpaceFillingTree();

    /*!
     * Do a depth first walk along the tree.
     * This is a pre-order, in-order AND post-order DFS.
     * A node is visited.
     * Then it's child.
     * Then the node again.
     * Then the next child.
     *    ...
     * Then the node again.
     * Then its parent.
     * etc.
     * \param visitor The external handler of each location of a node.
     */
    void walk(LocationVisitor& visitor) const;

    /*!
     * Create a nice picture of the cross fractal with optional numbers of a pre-order DFS.
     * This DFS is not the same as the order from \ref SpaceFillingTree::walk
     * 
     * \param[out] out the canvas in which to draw
     * \param output_dfs_order Whether to output the numbers of a pre-order DFS
     */
    void debugOutput(SVG& out, bool output_dfs_order = false) const;

    /*!
     * Check whether the tree is correct.
     * 
     * Only handled in debug mode.
     * 
     * Works by way of assertions in DEBUG mode.
     * \return Whether problems have been found
     */
    bool debugCheck() const;

    /*!
     * Direction of a line segment.
     * 
     * Combinations of (left|right) and (up|down)
     * in circular order.
     */
    enum Direction : unsigned char
    {
        LU = 0,
        RU = 1,
        RD = 2,
        LD = 3,
        DIRECTION_COUNT = 4
    };

    /*!
     * A node in the space filling tree
     */
    class  Node
    {
    public:
        Node* parent; //!< Parent node. Optional because of root
        unsigned int recursion_depth; //!< The recursion distance from root.
        unsigned int distance_depth; //!< The node distance from root.
        unsigned int total_depth; //!< The total depth the tree should have
        Point middle; //!< The location of the middle of this node. This is the middle of the square which is to be covered by the subtree of this node.
        Direction parent_to_here_direction; //!< The direction from the parent middle to this nodes middle.
        Node* children[4]; //!< connected junction points ordered on absolute direction

        /*!
         * Create a node with given parameters and no children.
         * 
         * \param parent Parent node. Optional because of root
         * \param depth The number of generations BELOW this node. Not the distance from root.
         * \param middle The location of the middle of this node. This is the middle of the square which is to be covered by the subtree of this node.
         * \param parent_to_here_direction The direction from the parent middle to this nodes middle.
         * \param total_depth The total depth the tree should have
         */
        Node(Node* parent, unsigned int depth, Point middle, Direction parent_to_here_direction, unsigned int total_depth);

        /*!
         * Destruct node and all it's children recursively
         */
        ~Node();

        /*!
         * Construct the sub-tree of this node recursively.
         * 
         * \param child_offset The horizontal and vertical offset from this middle to create a child
         */
        void construct(coord_t child_offset);

        /*!
         * Construct the child and its sub-tree of a given direction.
         * 
         * \param direction The direction in which to construct a child
         * \param child_offset The horizontal and vertical offset from this middle to create a child
         */
        void constructNode(Direction direction, coord_t child_offset);

        /*!
         * Remove points along straight edges in the sub-tree of this node, recursively.
         */
        void prune();

        /*!
         * Walk through this subtree and set the distance_depth of each child lower than my own.
         */
        void setDistanceDepth();

        /*!
         * Do a pre-order, in-order and post-order walk of the subtree of this node.
         * \param visitor The external handler of each location of a node.
         */
        void walk(LocationVisitor& visitor) const;

        /*!
         * Create a nice picture of the cubtree of this node with optional numbers of a pre-order DFS.
         * This DFS is not the same as the order from \ref SpaceFillingTree::walk
         * 
         * \param[out] out the canvas in which to draw
         * \param parent_middle The location of the middle of the parent node
         * \param output_dfs_order Whether to output the numbers of a pre-order DFS
         * \param[in,out] order_nr The dfs order
         * \param output_directions Include numbers along each edge of th direction of that edge. This is handy to verify that all edges are pointing away from the root, rather than toward.
         * \param output_distance_depth Whether to output the distance depth of each node
         */
        void debugOutput(SVG& out, Point parent_middle, bool output_dfs_order, int& order_nr, bool output_directions = false, bool output_distance_depth = true) const;

        /*!
         * Check whether node is correct and recurse for all children.
         * 
         * Only processed in DEBUG mode.
         * 
         * Works by way of assertions in DEBUG mode.
         * \return Whether problems have been found
         */
        bool debugCheck() const;
    };

protected:
    AABB aabb; //!< The aabb of the pattern to be generated. This is the area which is covered by the limit of the fractal.
    Node* root; //!< The root node of the tree. The middle of the fractal.
    unsigned int depth; //!< The depth of this tree.
};
} // namespace cura


#endif // UTILS_SPACE_FILLING_TREE_H
