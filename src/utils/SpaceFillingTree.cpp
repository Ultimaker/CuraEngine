//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
#include "SpaceFillingTree.h"

namespace cura {


SpaceFillingTree::SpaceFillingTree(
    Point middle,
    coord_t radius,
    unsigned int depth)
: root(nullptr)
, depth(depth)
{
    unsigned int root_depth = 0;
    root = new Node(nullptr
    , root_depth
    , middle
    , Direction::DIRECTION_COUNT // The root has no parent to child direction, so DIRECTION_COUNT is used as exception value
    , depth
    );

    // total width = radius because 1 + .5 + .25 + ... = 2
    // therefore the initial width = .5 * radius
    coord_t first_offset = radius / 2; // horizontal/vertical distance from the middle to the end of the first line segment
    // construct 1st order children
    root->construct(first_offset);
    root->prune();
    root->distance_depth = 0;
    root->setDistanceDepth();
}


SpaceFillingTree::SpaceFillingTree(SpaceFillingTree&& other)
: aabb(other.aabb)
, root(other.root)
{ // move constructor
    other.root = nullptr;
}

SpaceFillingTree::~SpaceFillingTree()
{
    if (root)
    {
        delete root;
    }
}

void SpaceFillingTree::walk(SpaceFillingTree::LocationVisitor& visitor) const
{
    root->walk(visitor);
}

void SpaceFillingTree::debugOutput(SVG& out, bool output_dfs_order) const
{
    out.writePolygon(aabb.toPolygon());
    int root_order = 0;
    root->debugOutput(out, root->middle, output_dfs_order, root_order); // root will draw line from its middle to the same middle
}

bool SpaceFillingTree::debugCheck() const
{
    return root->debugCheck();
}

SpaceFillingTree::Node::Node(SpaceFillingTree::Node* parent, unsigned int recursion_depth, Point middle, Direction parent_to_here_direction, unsigned int total_depth)
: parent(parent)
, recursion_depth(recursion_depth)
, total_depth(total_depth)
, middle(middle)
, parent_to_here_direction(parent_to_here_direction)
{
    for (Node*& child_place : children)
    {
        child_place = nullptr;
    }
}

SpaceFillingTree::Node::~Node()
{
    for (Node* child : children)
    {
        if (child)
        {
            delete child;
        }
    }
}

void SpaceFillingTree::Node::construct(coord_t child_offset)
{
    for (Direction child_dir = static_cast<Direction>(0); child_dir < Direction::DIRECTION_COUNT; child_dir = static_cast<Direction>(child_dir + 1))
    {
        constructNode(child_dir, child_offset);
    }
}

void SpaceFillingTree::Node::constructNode(Direction direction, coord_t child_offset)
{
    Point child_middle_offset = Point(child_offset, child_offset);
    if (direction == Direction::LD || direction == Direction::RD)
    { // if down
        child_middle_offset.Y *= -1;
    }
    if (direction == Direction::LD || direction == Direction::LU)
    { // if left
        child_middle_offset.X *= -1;
    }
    const Point child_middle = middle + child_middle_offset;
    Node* new_node;
    if (parent && direction == (parent_to_here_direction + 2) % Direction::DIRECTION_COUNT)
    {
        /*
         * reorder:
         *   make
         * parent-->new_node-->this
         *   rather than
         * parent------------->this
         *         new_node<---'
         */
        new_node = new Node(parent, recursion_depth + 1, child_middle, parent_to_here_direction, total_depth);
        parent->children[parent_to_here_direction] = new_node;
        new_node->children[parent_to_here_direction] = this;
        parent = new_node;
    }
    else
    {
        new_node = new Node(this, recursion_depth + 1, child_middle, direction, total_depth);
        if (children[direction])
        {
            /*!
             * reorder
             *  from
             * this------------->child
             *   make
             * this-->new_node-->child
             */
            new_node->children[direction] = this->children[direction];
            new_node->children[direction]->parent = new_node;
        }
        this->children[direction] = new_node;
    }
#ifdef DEBUG
    if (parent)
    {
        parent->debugCheck();
    }
    else
    {
        debugCheck();
    }
#endif // DEBUG

    assert(new_node->parent == this || new_node == parent);
    if (recursion_depth >= total_depth)
    {
        return;
    }
    new_node->construct(child_offset / 2);
}

void SpaceFillingTree::Node::prune()
{
    for (int child_dir = 0; child_dir < Direction::DIRECTION_COUNT; child_dir++)
    {
        Node*& child = children[child_dir];
        if (!child)
        {
            continue;
        }
        Node* right = child->children[(child_dir + 1) % Direction::DIRECTION_COUNT];
        Node* front = child->children[child_dir];
        Node* left = child->children[(child_dir + 3) % Direction::DIRECTION_COUNT];
        if (front && !left && !right)
        {
            Node* original_child = child;
            child = front; // move grandchild to children
            child->parent = this;
            original_child->children[child_dir] = nullptr; // so that it won't be deleted
            assert(!original_child->children[0]);
            assert(!original_child->children[1]);
            assert(!original_child->children[2]);
            assert(!original_child->children[3]);
            delete original_child;
            child_dir--; // make the next iteration of this loop handle this new child again
#ifdef DEBUG
            debugCheck();
#endif // DEBUG
        }
        child->prune(); // prune new or existing child recursively
        assert(child->parent == this);
    }
}

void SpaceFillingTree::Node::setDistanceDepth()
{
    for (int child_dir = 0; child_dir < Direction::DIRECTION_COUNT; child_dir++)
    {
        Node*& child = children[child_dir];
        if (!child)
        {
            continue;
        }
        child->distance_depth = distance_depth + 1;
        child->setDistanceDepth();
    }
}

void SpaceFillingTree::Node::walk(SpaceFillingTree::LocationVisitor& visitor) const
{
    visitor.visit(this);
    for (int dir_offset = 0; dir_offset < Direction::DIRECTION_COUNT; dir_offset++)
    {
        int direction = (parent_to_here_direction + dir_offset + 2) % Direction::DIRECTION_COUNT;
        Node* child = children[direction];
        if (child)
        {
            assert(child->parent == this);
            child->walk(visitor);
            visitor.visit(this);
        }
    }
}

void SpaceFillingTree::Node::debugOutput(SVG& out, Point parent_middle, bool output_dfs_order, int& order_nr, bool output_directions, bool output_distance_depth) const
{
    out.writeLine(parent_middle, middle);
    if (output_dfs_order)
    {
        out.writeText(middle, std::to_string(order_nr));
    }
    if (output_distance_depth)
    {
        out.writeText(middle, std::to_string(distance_depth));
    }
    for (int dir_offset = 0; dir_offset < Direction::DIRECTION_COUNT; dir_offset++)
    {
        int direction = (parent_to_here_direction + dir_offset + 2) % Direction::DIRECTION_COUNT;
        Node* child = children[direction];
        if (child)
        {
            order_nr++;
            if (output_directions)
            {
                out.writeText((middle + child->middle) / 2, std::to_string(direction), SVG::Color::BLUE);
            }
            child->debugOutput(out, middle, output_dfs_order, order_nr, output_directions, output_distance_depth);
        }
    }
}

bool SpaceFillingTree::Node::debugCheck() const
{
    bool fail = false;
    for (int child_dir = 0; child_dir < Direction::DIRECTION_COUNT; child_dir++)
    {
        Node* child = children[child_dir];
        if (child)
        {
#ifdef DEBUG
            assert(child->children[(child_dir + 2) % Direction::DIRECTION_COUNT] == nullptr);
            assert(child->parent == this);
            assert(child->parent_to_here_direction == static_cast<Direction>(child_dir));
#endif // DEBUG
            fail |= child->children[(child_dir + 2) % Direction::DIRECTION_COUNT] != nullptr;
            fail |= child->parent != this;
            fail |= child->parent_to_here_direction != static_cast<Direction>(child_dir);
            fail |= child->debugCheck();
            assert(!fail);
        }
    }
    return fail;
}

}; // namespace cura
