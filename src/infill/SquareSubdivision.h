#ifndef INFILL_SQUARE_SUBDIVISION_H
#define INFILL_SQUARE_SUBDIVISION_H

#include <functional> // function

#include "../utils/AABB.h"
#include "../utils/Range.h"
#include "../utils/SVG.h"
#include "InfillFractal.h"
#include "SubdivSquare.h"

namespace cura
{



class SquareSubdivision : public InfillFractal<AABB, 4, 2>
{
    using Parent = InfillFractal<AABB, 4, 2>;
public:
    SquareSubdivision(const DensityProvider& density_provider, coord_t line_width)
    : Parent(density_provider, line_width)
    {
    }
protected:
    float getActualizedArea(const Parent::Cell& node)
    {
        const AABB& aabb = node.elem;
        Point diagonal = aabb.max - aabb.min;
        return INT2MM(Parent::line_width) * INT2MM(diagonal.X + diagonal.Y);
    }
    
    enum class Direction : int
    {
        LEFT = 0, // ordered on polarity and dimension: first X from less to more, then Y
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };
    Direction opposite(Direction in)
    {
        switch(in)
        {
            case Direction::LEFT: return Direction::RIGHT;
            case Direction::RIGHT: return Direction::LEFT;
            case Direction::UP: return Direction::DOWN;
            case Direction::DOWN: return Direction::UP;
            default: return Direction::COUNT;
        }
    }
    
    void createTree(Cell& sub_tree_root, int max_depth)
    {
        int parent_depth = sub_tree_root.depth;
        if (parent_depth >= max_depth)
        {
            return;
        }
        
        const AABB& parent_aabb = sub_tree_root.elem;
        Point middle = parent_aabb.getMiddle();
        Cell* child_lb = new Cell(AABB(parent_aabb.min, middle), parent_depth + 1, number_of_sides);
        Cell* child_rb = new Cell(AABB(Point(middle.X, parent_aabb.min.Y), Point(parent_aabb.max.X, middle.Y)), parent_depth + 1, number_of_sides);
        Cell* child_lt = new Cell(AABB(Point(parent_aabb.min.X, middle.Y), Point(middle.X, parent_aabb.max.Y)), parent_depth + 1, number_of_sides);
        Cell* child_rt = new Cell(AABB(middle, parent_aabb.max), parent_depth + 1, number_of_sides);
        sub_tree_root.children[0] = child_lb; // ordered on polarity and dimension: first X from less to more, then Y
        sub_tree_root.children[1] = child_rb;
        sub_tree_root.children[2] = child_lt;
        sub_tree_root.children[3] = child_rt;
        
        for (Cell* child : sub_tree_root.children)
        {
            createTree(*child, max_depth);
        }
    }
    
    void getSpecificationAllowance(Cell& sub_tree_root)
    {
        Point area_dimensions = sub_tree_root.elem.max - sub_tree_root.elem.min;
        sub_tree_root.area = INT2MM(area_dimensions.X) * INT2MM(area_dimensions.Y);
        
        bool has_children = sub_tree_root.children[0] != nullptr;
        if (has_children)
        {
            for (Cell* child : sub_tree_root.children)
            {
                getSpecificationAllowance(*child);
                sub_tree_root.filled_area_allowance += child->filled_area_allowance;
            }
        }
        else
        {
            float requested_density = density_provider(sub_tree_root.elem);
            sub_tree_root.filled_area_allowance = sub_tree_root.area * requested_density;
        }
    }
    
    virtual void createTree()
    {
        int max_depth = 10; // TODO
        assert(root && "Root must be initialized when building the tree");
        createTree(*root, max_depth);
        
        getSpecificationAllowance(*root);
    }
    
    
    static constexpr char number_of_sides = 4;
    
    
    void initialConnection(Cell* before, Cell* after, Direction dir)
    {
        before->adjacent_cells[static_cast<size_t>(dir)].emplace_front(after, nullptr);
        after->adjacent_cells[static_cast<size_t>(opposite(dir))].emplace_front(before, nullptr);
        Link& before_to_after = before->adjacent_cells[static_cast<size_t>(dir)].front();
        Link& after_to_before = after->adjacent_cells[static_cast<size_t>(opposite(dir))].front();
        before_to_after.reverse = &after_to_before;
        after_to_before.reverse = &before_to_after;
    }
    
    /*!
     * 
     * \param dimension The dimension to check whether they are next to each other. 0 for X if you want to check if the cells are besides each other, 1 for on top of each other.
     */
    bool isNextTo(const Cell& a, const Cell& b, int dimension)
    {
        AABB a_square_expanded = a.elem;
        a_square_expanded.expand(10); // TODO: move allowed error to central place?
        if (!a_square_expanded.hit(b.elem))
        {
            return false; // cells are apart:  []     []
        }
        Range<coord_t> a_range(getCoord(a.elem.min, !dimension), getCoord(a.elem.max, !dimension));
        Range<coord_t> b_range(getCoord(b.elem.min, !dimension), getCoord(b.elem.max, !dimension));
        // contract to prevent
        // []
        //   []   from counting as being next to each other
        return a_range.expand(-10).overlap(b_range); // TODO: move allowed error to central place?
    }
    
    void subdivide(Cell* cell)
    {
        if (!cell)
        {
            assert(false && "Subdivide called on nullptr!");
            return;
        }
        
        Cell* child_lb = cell->children[0];
        Cell* child_rb = cell->children[1];
        Cell* child_lt = cell->children[2];
        Cell* child_rt = cell->children[3];
        
        assert(child_lb && child_lt && child_rb && child_rt && "Children must be initialized for subdivision!");
        
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_rb, Direction::RIGHT);
        initialConnection(child_lt, child_lb, Direction::UP);
        initialConnection(child_rt, child_rb, Direction::UP);
        
        for (int side = 0; side < number_of_sides; side++)
        {
            int edge_dim = !(side / 2);
            
            for (Link& neighbor : cell->adjacent_cells[side])
            {
                for (Cell* child : cell->children)
                {
                    assert(child);
                    assert(neighbor.to);
                    if (isNextTo(*child, *neighbor.to, !edge_dim))
                    {
                        neighbor.reverse->to = child; // TODO: should insert each neighboring child!
                    }
                }
            }
            
        }
    }
    
    
    
    void debugOutput(SVG& svg)
    {
        
    }
};

}
#endif //INFILL_SQUARE_SUBDIVISION_H
