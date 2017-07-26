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

/*!
 * TODO
 */
class SpaceFillingTree
{
public:
    class LocationVisitor
    {
    public:
        virtual void visit(Point junction) = 0;
    };
    SpaceFillingTree(
        Point middle,
        coord_t radius,
        int depth);

    SpaceFillingTree(SpaceFillingTree&& other);

    ~SpaceFillingTree();

    void walk(LocationVisitor& visitor) const;

    void debugOutput(SVG& out, bool output_dfs_order = false);

    void debugCheck();
protected:
    /*!
     * TODO
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
     * TODO
     */
    class  Node
    {
    public:
        Node* parent; // optional because of root
        int depth;
        Point middle;
        Direction parent_to_here_direction;
        Node* children[4]; //!< connected junction points ordered on absolute direction

        Node(Node* parent, int depth, Point middle, Direction parent_to_here_direction);

        ~Node();

        void addChild(Direction direction, coord_t child_offset);

        void prune();

        void walk(LocationVisitor& visitor) const;

        void debugOutput(SVG& out, Point parent_middle, bool output_dfs_order, int& order_nr, bool output_directions = false);

        void debugCheck();
    };
    AABB aabb;
    Node* root;
};
} // namespace cura


#endif // UTILS_SPACE_FILLING_TREE_H
